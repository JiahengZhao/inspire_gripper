#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "inspire_hand/serial_bus.hpp"

namespace inspire_hand {

struct JointState {
  uint8_t gripper_id{1};
  double default_speed{500.0};
  double pos_cmd{std::numeric_limits<double>::quiet_NaN()};
  double eff_cmd{std::numeric_limits<double>::quiet_NaN()};
  double pos_state{0.0};
  double vel_state{0.0};
  double eff_state{0.0};
  double last_pos_written{std::numeric_limits<double>::quiet_NaN()};
  double last_eff_written{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::time_point last_read_time{};
  double last_pos_state{0.0};
};

class GripperSystem : public hardware_interface::SystemInterface {
public:
  GripperSystem();
  ~GripperSystem() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;

  std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

  // Exposed for Task 13 (services.cpp): bus, joints, shared mutex.
  SerialBus& bus() { return bus_; }
  std::vector<JointState>& joints() { return joints_; }
  std::mutex& state_mutex() { return state_mu_; }

  bool allow_set_id() const noexcept { return allow_set_id_; }
  int read_timeout_ms() const noexcept { return read_timeout_ms_; }

  // URDF joint 0 rad = gripper fully open (matches raw 1000).
  // URDF joint kJointMaxRad = gripper fully closed (matches raw 0).
  static constexpr double kJointMaxRad = 0.8663;
  static constexpr uint16_t kMaxRaw    = 1000;

private:
  std::vector<JointState> joints_;
  SerialBus bus_;
  std::mutex state_mu_;

  std::string port_{"/dev/ttyUSB0"};
  int baud_{115200};
  int read_timeout_ms_{30};
  uint16_t default_force_threshold_g_{500};
  bool allow_set_id_{false};

  void start_services();
  void stop_services();

  double raw_to_joint(uint16_t raw) const {
    return kJointMaxRad * (1.0 - static_cast<double>(raw) / kMaxRaw);
  }
  uint16_t joint_to_raw(double rad) const {
    double r = kMaxRaw * (1.0 - rad / kJointMaxRad);
    if (r < 0.0) return 0;
    if (r > kMaxRaw) return kMaxRaw;
    return static_cast<uint16_t>(r + 0.5);
  }
};

}  // namespace inspire_hand
