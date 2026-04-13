#include "inspire_hand/gripper_system.hpp"
#include "inspire_hand/protocol.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "inspire_hand/srv/stop.hpp"
#include "inspire_hand/srv/clear_fault.hpp"
#include "inspire_hand/srv/save_params.hpp"
#include "inspire_hand/srv/set_id.hpp"
#include "inspire_hand/srv/calibrate_range.hpp"
#include "inspire_hand/srv/set_force_threshold.hpp"
#include "inspire_hand/srv/read_run_state.hpp"

namespace inspire_hand {

namespace {
std::string error_bits_to_string(uint8_t e) {
  if (e == 0) return "none";
  std::string s;
  auto add = [&](const char* n){ if (!s.empty()) s += ","; s += n; };
  if (e & 0x01) add("locked_rotor");
  if (e & 0x02) add("over_temp");
  if (e & 0x04) add("over_current");
  if (e & 0x08) add("driver_fault");
  if (e & 0x10) add("comms_failure");
  return s;
}

const char* run_state_name(uint8_t s) {
  switch (s) {
    case 0x01: return "unclenched_idle";
    case 0x02: return "clenched_idle";
    case 0x03: return "stopped_idle";
    case 0x04: return "clenching";
    case 0x05: return "unclenching";
    case 0x06: return "caught_object";
    default:   return "unknown";
  }
}

struct ServicesState {
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec;
  std::thread thread;
  std::atomic<bool> stop{false};

  rclcpp::Service<inspire_hand::srv::Stop>::SharedPtr               s_stop;
  rclcpp::Service<inspire_hand::srv::ClearFault>::SharedPtr         s_clear;
  rclcpp::Service<inspire_hand::srv::SaveParams>::SharedPtr         s_save;
  rclcpp::Service<inspire_hand::srv::SetId>::SharedPtr              s_setid;
  rclcpp::Service<inspire_hand::srv::CalibrateRange>::SharedPtr     s_cal;
  rclcpp::Service<inspire_hand::srv::SetForceThreshold>::SharedPtr  s_force;
  rclcpp::Service<inspire_hand::srv::ReadRunState>::SharedPtr       s_run;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub;
  rclcpp::TimerBase::SharedPtr diag_timer;
};

std::mutex g_svc_mu;
std::unordered_map<GripperSystem*, std::unique_ptr<ServicesState>> g_svc;

}  // namespace

void GripperSystem::start_services() {
  std::lock_guard<std::mutex> g(g_svc_mu);
  auto st = std::make_unique<ServicesState>();

  // Use a unique node name so multiple buses can coexist in the same process.
  static std::atomic<int> bus_counter{0};
  const int bus_idx = bus_counter.fetch_add(1);
  const std::string node_name = "inspire_hand_bus" + std::to_string(bus_idx);

  rclcpp::NodeOptions opts;
  st->node = std::make_shared<rclcpp::Node>(node_name, "inspire_hand", opts);

  const auto timeout = std::chrono::milliseconds(read_timeout_ms());
  auto& bus = bus_;
  auto* self = this;

  auto find_joint = [self](uint8_t id) -> JointState* {
    for (auto& jj : self->joints()) if (jj.gripper_id == id) return &jj;
    return nullptr;
  };

  st->s_stop = st->node->create_service<srv::Stop>(
      "stop",
      [&bus, timeout](const std::shared_ptr<srv::Stop::Request> req,
                      std::shared_ptr<srv::Stop::Response> res) {
        auto r = bus.transact(make_stop(req->gripper_id), timeout);
        res->success = r.has_value();
        res->message = r ? "ok" : "bus error";
      });

  st->s_clear = st->node->create_service<srv::ClearFault>(
      "clear_fault",
      [&bus, timeout](const std::shared_ptr<srv::ClearFault::Request> req,
                      std::shared_ptr<srv::ClearFault::Response> res) {
        auto r = bus.transact(make_clear_fault(req->gripper_id), timeout);
        res->success = r.has_value();
        res->message = r ? "ok" : "bus error";
      });

  st->s_save = st->node->create_service<srv::SaveParams>(
      "save_params",
      [&bus](const std::shared_ptr<srv::SaveParams::Request> req,
             std::shared_ptr<srv::SaveParams::Response> res) {
        auto r = bus.transact(make_para_save(req->gripper_id), std::chrono::milliseconds(1500));
        res->success = r.has_value();
        res->message = r ? "ok" : "bus error";
      });

  const bool allow_set_id_local = allow_set_id();
  st->s_setid = st->node->create_service<srv::SetId>(
      "set_id",
      [&bus, timeout, allow_set_id_local](const std::shared_ptr<srv::SetId::Request> req,
                                          std::shared_ptr<srv::SetId::Response> res) {
        if (!allow_set_id_local) {
          res->success = false;
          res->message = "disabled (set allow_set_id=true in URDF)";
          return;
        }
        auto r = bus.transact(make_para_id_set(req->gripper_id, req->new_id), timeout);
        res->success = r.has_value();
        res->message = r ? "ok; call save_params to persist" : "bus error";
      });

  st->s_cal = st->node->create_service<srv::CalibrateRange>(
      "calibrate_range",
      [&bus, timeout](const std::shared_ptr<srv::CalibrateRange::Request> req,
                      std::shared_ptr<srv::CalibrateRange::Response> res) {
        auto r = bus.transact(make_set_eg_para(req->gripper_id, req->max_raw, req->min_raw), timeout);
        res->success = r.has_value();
        res->message = r ? "ok" : "bus error";
      });

  st->s_force = st->node->create_service<srv::SetForceThreshold>(
      "set_force_threshold",
      [self, find_joint](const std::shared_ptr<srv::SetForceThreshold::Request> req,
                         std::shared_ptr<srv::SetForceThreshold::Response> res) {
        std::lock_guard<std::mutex> gg(self->state_mutex());
        auto* j = find_joint(req->gripper_id);
        if (!j) {
          res->success = false; res->message = "unknown gripper_id";
          return;
        }
        if (req->threshold_g < 50 || req->threshold_g > 1000) {
          res->success = false; res->message = "threshold out of range [50,1000]";
          return;
        }
        j->eff_cmd = static_cast<double>(req->threshold_g);
        res->success = true;
        res->message = "stored; applied on next CATCH move";
      });

  st->s_run = st->node->create_service<srv::ReadRunState>(
      "read_run_state",
      [&bus, timeout](const std::shared_ptr<srv::ReadRunState::Request> req,
                      std::shared_ptr<srv::ReadRunState::Response> res) {
        auto r = bus.transact(make_read_eg_run(req->gripper_id), timeout);
        if (!r || r->data.size() < 7) {
          res->success = false; res->message = "bus error or short response";
          return;
        }
        res->success        = true;
        res->message        = run_state_name(r->data[0]);
        res->running_state  = r->data[0];
        res->error_code     = r->data[1];
        res->temperature_c  = r->data[2];
        res->opening_raw    = static_cast<uint16_t>(r->data[3]) | (static_cast<uint16_t>(r->data[4]) << 8);
        res->force_set_g    = static_cast<uint16_t>(r->data[5]) | (static_cast<uint16_t>(r->data[6]) << 8);
      });

  st->diag_pub = st->node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

  st->diag_timer = st->node->create_wall_timer(
      std::chrono::milliseconds(500),
      [self, &bus, timeout, pub = st->diag_pub]() {
        diagnostic_msgs::msg::DiagnosticArray arr;
        arr.header.stamp = rclcpp::Clock().now();
        for (auto& j : self->joints()) {
          auto r = bus.transact(make_read_eg_run(j.gripper_id), timeout);
          diagnostic_msgs::msg::DiagnosticStatus s;
          s.name = std::string("inspire_hand/gripper_") + std::to_string(j.gripper_id);
          s.hardware_id = "inspire_eg2_" + std::to_string(j.gripper_id);
          if (!r || r->data.size() < 7) {
            s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            s.message = "bus error";
          } else {
            const uint8_t state = r->data[0];
            const uint8_t err = r->data[1];
            if (err != 0) { s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR; s.message = "fault"; }
            else          { s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;    s.message = run_state_name(state); }
            auto kv = [&](const std::string& k, const std::string& v){
              diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = v; s.values.push_back(p);
            };
            kv("running_state",  run_state_name(state));
            kv("error_bits",     error_bits_to_string(err));
            kv("temperature_c",  std::to_string(r->data[2]));
            kv("opening_raw",    std::to_string(static_cast<uint16_t>(r->data[3]) | (static_cast<uint16_t>(r->data[4]) << 8)));
            kv("force_set_g",    std::to_string(static_cast<uint16_t>(r->data[5]) | (static_cast<uint16_t>(r->data[6]) << 8)));
          }
          arr.status.push_back(std::move(s));
        }
        pub->publish(arr);
      });

  st->exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  st->exec->add_node(st->node);
  st->thread = std::thread([exec = st->exec, stop = &st->stop](){
    while (!stop->load()) exec->spin_some(std::chrono::milliseconds(50));
  });

  g_svc[this] = std::move(st);
}

void GripperSystem::stop_services() {
  std::unique_ptr<ServicesState> st;
  {
    std::lock_guard<std::mutex> g(g_svc_mu);
    auto it = g_svc.find(this);
    if (it == g_svc.end()) return;
    st = std::move(it->second);
    g_svc.erase(it);
  }
  st->stop = true;
  if (st->thread.joinable()) st->thread.join();
  st->exec->remove_node(st->node);
}

}  // namespace inspire_hand
