// Copyright 2024 ros2_lingua contributors
// SPDX-License-Identifier: Apache-2.0
//
// ros2_lingua_cpp/lingua_mixin.hpp
// ----------------------------------
// C++ equivalent of Python's LinguaMixin.
//
// Usage:
//   class MyNode : public lingua::LinguaMixin<rclcpp::Node> {
//   public:
//     MyNode() : LinguaMixin("my_node") {
//       lingua::Capability cap;
//       cap.name = "do_something";
//       cap.description = "Does something cool";
//       cap.ros_action = "robot/do_something";
//       register_capability(cap);
//     }
//   };

#ifndef ROS2_LINGUA_CPP__LINGUA_MIXIN_HPP_
#define ROS2_LINGUA_CPP__LINGUA_MIXIN_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

#include "ros2_lingua_interfaces/srv/register_capability.hpp"
#include "ros2_lingua_interfaces/srv/update_state.hpp"

#include "ros2_lingua_cpp/capability.hpp"
#include "ros2_lingua_cpp/json.hpp"

namespace lingua {

/// Template mixin that adds ros2_lingua capability registration to any
/// rclcpp::Node-derived class.
///
/// @tparam NodeT  Base node type (typically rclcpp::Node)
template <typename NodeT = rclcpp::Node>
class LinguaMixin : public NodeT {
public:
  using RegisterCapability = ros2_lingua_interfaces::srv::RegisterCapability;
  using UpdateState = ros2_lingua_interfaces::srv::UpdateState;

  /// Construct the mixin.  Forwards all arguments to NodeT's constructor.
  template <typename... Args>
  explicit LinguaMixin(Args &&... args)
  : NodeT(std::forward<Args>(args)...)
  {
    register_client_ = this->template create_client<RegisterCapability>(
      "lingua/register_capability");
    state_client_ = this->template create_client<UpdateState>(
      "lingua/update_state");

    reregister_sub_ = this->template create_subscription<std_msgs::msg::Empty>(
      "lingua/request_reregister", 10,
      [this](const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        handle_reregister_request();
      });
  }

  /// Register a capability with the GroundingNode.
  ///
  /// Retries up to @p max_retries times with exponential backoff if the
  /// GroundingNode service isn't available yet.
  ///
  /// @param cap          The Capability to register
  /// @param timeout_sec  Seconds to wait for the service per attempt
  /// @param max_retries  Number of retry attempts before giving up
  /// @return true on success, false on failure
  bool register_capability(
    const Capability & cap,
    double timeout_sec = 10.0,
    int max_retries = 3)
  {
    // Track for re-registration
    bool already_tracked = false;
    for (const auto & c : registered_capabilities_) {
      if (c.name == cap.name) {
        already_tracked = true;
        break;
      }
    }
    if (!already_tracked) {
      registered_capabilities_.push_back(cap);
    }

    return execute_registration(cap, timeout_sec, max_retries);
  }

  /// Notify the GroundingNode of a symbolic state change.
  ///
  /// @param set_tokens    State tokens to mark as True
  /// @param clear_tokens  State tokens to mark as False
  /// @param timeout_sec   How long to wait for the service
  /// @return true on success, false on failure (non-fatal)
  bool update_state(
    const std::vector<std::string> & set_tokens = {},
    const std::vector<std::string> & clear_tokens = {},
    double timeout_sec = 3.0)
  {
    if (set_tokens.empty() && clear_tokens.empty()) {
      return true;  // no-op
    }

    if (!state_client_->wait_for_service(
        std::chrono::duration<double>(2.0)))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "[Lingua] lingua/update_state not available — state update skipped.");
      return false;
    }

    nlohmann::json j;
    j["set"] = set_tokens;
    j["clear"] = clear_tokens;

    auto request = std::make_shared<UpdateState::Request>();
    request->state_json = j.dump();

    auto future = state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), future,
        std::chrono::duration<double>(timeout_sec)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "[Lingua] State update timed out. set=%zu, clear=%zu",
        set_tokens.size(), clear_tokens.size());
      return false;
    }

    auto result = future.get();
    if (!result->success) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Lingua] State update failed: %s", result->message.c_str());
    }
    return result->success;
  }

protected:
  /// Called when the GroundingNode requests re-registration.
  void handle_reregister_request()
  {
    if (registered_capabilities_.empty()) {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[Lingua] Grounding node requested re-registration. "
      "Re-registering %zu capabilities...",
      registered_capabilities_.size());

    for (const auto & cap : registered_capabilities_) {
      execute_registration(cap, 10.0, 3);
    }
  }

private:
  /// Internal helper: execute registration with retries + backoff.
  bool execute_registration(
    const Capability & cap,
    double timeout_sec,
    int max_retries)
  {
    double delay = 1.0;

    for (int attempt = 1; attempt <= max_retries; ++attempt) {
      if (!register_client_->wait_for_service(
          std::chrono::duration<double>(timeout_sec)))
      {
        if (attempt < max_retries) {
          RCLCPP_WARN(
            this->get_logger(),
            "[Lingua] lingua/register_capability not available "
            "(attempt %d/%d). Retrying in %.0fs...",
            attempt, max_retries, delay);
          std::this_thread::sleep_for(
            std::chrono::duration<double>(delay));
          delay *= 2.0;
          continue;
        } else {
          RCLCPP_ERROR(
            this->get_logger(),
            "[Lingua] Could not reach lingua/register_capability "
            "after %d attempts. Is the GroundingNode running?",
            max_retries);
          return false;
        }
      }

      auto request = std::make_shared<RegisterCapability::Request>();
      request->capability_json = cap.to_json();

      auto future = register_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(
          this->get_node_base_interface(), future,
          std::chrono::duration<double>(timeout_sec)) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        if (attempt < max_retries) {
          RCLCPP_WARN(
            this->get_logger(),
            "[Lingua] Registration of '%s' timed out "
            "(attempt %d/%d). Retrying...",
            cap.name.c_str(), attempt, max_retries);
          std::this_thread::sleep_for(
            std::chrono::duration<double>(delay));
          delay *= 2.0;
          continue;
        } else {
          RCLCPP_ERROR(
            this->get_logger(),
            "[Lingua] Registration of '%s' timed out "
            "after %d attempts.",
            cap.name.c_str(), max_retries);
          return false;
        }
      }

      auto result = future.get();
      if (result->success) {
        RCLCPP_INFO(
          this->get_logger(),
          "Registered capability: '%s'", cap.name.c_str());
        return true;
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "[Lingua] Registration rejected for '%s': %s",
          cap.name.c_str(), result->message.c_str());
        return false;  // Rejection is not worth retrying
      }
    }

    return false;
  }

  // --- Members ---
  typename rclcpp::Client<RegisterCapability>::SharedPtr register_client_;
  typename rclcpp::Client<UpdateState>::SharedPtr state_client_;
  typename rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reregister_sub_;
  std::vector<Capability> registered_capabilities_;
};

}  // namespace lingua

#endif  // ROS2_LINGUA_CPP__LINGUA_MIXIN_HPP_
