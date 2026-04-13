#include "inspire_hand/gripper_system.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/clock.hpp>

#include <algorithm>
#include <cmath>

namespace inspire_hand {

using CallbackReturn = hardware_interface::CallbackReturn;

GripperSystem::GripperSystem() = default;

GripperSystem::~GripperSystem() {
  stop_services();
  bus_.close();
}

CallbackReturn GripperSystem::on_init(const hardware_interface::HardwareInfo& info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

  auto get = [](const auto& map, const std::string& key, const std::string& fallback) {
    auto it = map.find(key);
    return it == map.end() ? fallback : it->second;
  };

  port_                      = get(info.hardware_parameters, "port", "/dev/ttyUSB0");
  baud_                      = std::stoi(get(info.hardware_parameters, "baudrate", "115200"));
  read_timeout_ms_           = std::stoi(get(info.hardware_parameters, "read_timeout_ms", "30"));
  default_force_threshold_g_ = static_cast<uint16_t>(std::stoi(
      get(info.hardware_parameters, "default_force_threshold", "500")));
  allow_set_id_ = (get(info.hardware_parameters, "allow_set_id", "false") == "true");

  joints_.clear();
  joints_.reserve(info.joints.size());
  for (const auto& j : info.joints) {
    JointState js;
    js.gripper_id    = static_cast<uint8_t>(std::stoi(get(j.parameters, "gripper_id", "1")));
    js.default_speed = std::stod(get(j.parameters, "default_speed", "500"));

    bool has_pos_cmd = false, has_eff_cmd = false;
    for (const auto& ci : j.command_interfaces) {
      if (ci.name == "position") has_pos_cmd = true;
      if (ci.name == "effort")   has_eff_cmd = true;
    }
    if (!has_pos_cmd) {
      RCLCPP_ERROR(rclcpp::get_logger("inspire_hand"),
                   "Joint '%s' must declare a 'position' command interface.", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (!has_eff_cmd) {
      RCLCPP_WARN(rclcpp::get_logger("inspire_hand"),
                  "Joint '%s' has no 'effort' command interface; force threshold will use default.",
                  j.name.c_str());
    }
    joints_.push_back(js);
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GripperSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto& name = info_.joints[i].name;
    out.emplace_back(name, "position", &joints_[i].pos_state);
    out.emplace_back(name, "velocity", &joints_[i].vel_state);
    out.emplace_back(name, "effort",   &joints_[i].eff_state);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> GripperSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto& name = info_.joints[i].name;
    out.emplace_back(name, "position", &joints_[i].pos_cmd);
    out.emplace_back(name, "effort",   &joints_[i].eff_cmd);
  }
  return out;
}

CallbackReturn GripperSystem::on_configure(const rclcpp_lifecycle::State&) {
  auto r = bus_.open(port_, baud_);
  if (!r) {
    RCLCPP_ERROR(rclcpp::get_logger("inspire_hand"),
                 "Failed to open %s @ %d", port_.c_str(), baud_);
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperSystem::on_activate(const rclcpp_lifecycle::State&) {
  const auto timeout = std::chrono::milliseconds(read_timeout_ms_);
  for (auto& j : joints_) {
    auto resp = bus_.transact(make_read_act_pos(j.gripper_id), timeout);
    if (resp && resp->data.size() == 2) {
      const uint16_t raw = static_cast<uint16_t>(resp->data[0]) |
                           (static_cast<uint16_t>(resp->data[1]) << 8);
      j.pos_state = raw_to_joint(raw);
    }
    j.pos_cmd          = j.pos_state;
    j.eff_cmd          = static_cast<double>(default_force_threshold_g_);
    j.last_pos_written = std::numeric_limits<double>::quiet_NaN();
    j.last_eff_written = std::numeric_limits<double>::quiet_NaN();
    j.last_read_time   = std::chrono::steady_clock::now();
    j.last_pos_state   = j.pos_state;
  }
  start_services();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperSystem::on_deactivate(const rclcpp_lifecycle::State&) {
  stop_services();
  const auto timeout = std::chrono::milliseconds(read_timeout_ms_);
  for (auto& j : joints_) (void)bus_.transact(make_stop(j.gripper_id), timeout);
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperSystem::on_cleanup(const rclcpp_lifecycle::State&) {
  bus_.close();
  return CallbackReturn::SUCCESS;
}

// --- Task 10: read() ---
hardware_interface::return_type GripperSystem::read(const rclcpp::Time&, const rclcpp::Duration&) {
  const auto timeout = std::chrono::milliseconds(read_timeout_ms_);
  const auto now = std::chrono::steady_clock::now();
  static auto clk = rclcpp::Clock::make_shared();

  std::lock_guard<std::mutex> g(state_mu_);
  for (auto& j : joints_) {
    auto resp = bus_.transact(make_read_eg_state(j.gripper_id), timeout);
    if (!resp) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("inspire_hand"), *clk, 2000,
        "read gripper %u failed (err=%d)", j.gripper_id, static_cast<int>(resp.error()));
      continue;
    }
    if (resp->data.size() < 6) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("inspire_hand"), *clk, 2000,
        "short ReadEgState response for id %u (%zu bytes)",
        j.gripper_id, resp->data.size());
      continue;
    }
    const uint16_t opening = static_cast<uint16_t>(resp->data[0]) |
                             (static_cast<uint16_t>(resp->data[1]) << 8);
    const uint16_t force   = static_cast<uint16_t>(resp->data[2]) |
                             (static_cast<uint16_t>(resp->data[3]) << 8);
    const double pos_rad = raw_to_joint(opening);
    const auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - j.last_read_time).count();
    j.vel_state      = (dt > 1e-3) ? (pos_rad - j.last_pos_state) / dt : 0.0;
    j.last_pos_state = pos_rad;
    j.last_read_time = now;
    j.pos_state      = pos_rad;
    j.eff_state      = static_cast<double>(force);
  }
  return hardware_interface::return_type::OK;
}

// --- Task 11: write() ---
hardware_interface::return_type GripperSystem::write(const rclcpp::Time&, const rclcpp::Duration&) {
  const auto timeout = std::chrono::milliseconds(read_timeout_ms_);
  static auto clk = rclcpp::Clock::make_shared();

  std::lock_guard<std::mutex> g(state_mu_);
  for (auto& j : joints_) {
    if (std::isnan(j.pos_cmd)) continue;

    const double tgt_rad = std::clamp(j.pos_cmd, 0.0, kJointMaxRad);
    const uint16_t tgt_raw = joint_to_raw(tgt_rad);

    double eff_g = j.eff_cmd;
    const bool use_force = !std::isnan(eff_g) && eff_g >= 50.0;
    if (use_force) eff_g = std::min(eff_g, 1000.0);
    else           eff_g = 0.0;

    const bool pos_same = !std::isnan(j.last_pos_written) &&
                          std::abs(j.last_pos_written - tgt_rad) < 1e-4;
    const bool eff_same = !std::isnan(j.last_eff_written) &&
                          std::abs(j.last_eff_written - eff_g) < 1.0;
    if (pos_same && eff_same) continue;

    const uint16_t speed = static_cast<uint16_t>(std::clamp(j.default_speed, 1.0, 1000.0));
    const double current_rad = j.pos_state;

    std::expected<Frame, BusError> rc;
    if (use_force && tgt_rad > current_rad + 1e-4) {
      rc = bus_.transact(make_move_catch(j.gripper_id, speed, static_cast<uint16_t>(eff_g)),
                         timeout);
    } else {
      rc = bus_.transact(make_seek_pos(j.gripper_id, tgt_raw), timeout);
    }
    if (!rc) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("inspire_hand"), *clk, 2000,
        "write gripper %u failed (err=%d)", j.gripper_id, static_cast<int>(rc.error()));
      continue;
    }
    j.last_pos_written = tgt_rad;
    j.last_eff_written = eff_g;
  }
  return hardware_interface::return_type::OK;
}


}  // namespace inspire_hand

PLUGINLIB_EXPORT_CLASS(inspire_hand::GripperSystem, hardware_interface::SystemInterface)
