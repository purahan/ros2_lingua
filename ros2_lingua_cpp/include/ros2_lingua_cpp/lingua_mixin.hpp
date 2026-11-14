// Copyright 2024 ros2_lingua contributors
// SPDX-License-Identifier: Apache-2.0
//
// ros2_lingua_cpp/lingua_mixin.hpp
// ----------------------------------
// C++ equivalent of Python's LinguaMixin.
//
// Before the node is added to an executor (e.g. in the constructor),
// register_capability() and update_state() block until they get a reply.
// Once the node is in an executor they send asynchronously, so they are
// safe to call from callbacks. Spinning from inside a callback throws
// "Node has already been added to an executor".
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

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

#include "ros2_lingua_interfaces/srv/register_capability.hpp"
#include "ros2_lingua_interfaces/srv/update_state.hpp"

#include "ros2_lingua_cpp/capability.hpp"
#include <nlohmann/json.hpp>

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
  /// Once the node is spinning, the registration is sent asynchronously and
  /// the result is logged.
  ///
  /// @param cap          The Capability to register
  /// @param timeout_sec  Seconds to wait for the service per attempt
  /// @param max_retries  Number of retry attempts before giving up
  /// @return true on success (or when sent asynchronously), false on failure
  bool register_capability(
    const Capability & cap,
    double timeout_sec = 10.0,
    int max_retries = 3)
  {
    // Track for re-registration, keeping only the latest definition per name
    registered_capabilities_.erase(
      std::remove_if(
        registered_capabilities_.begin(), registered_capabilities_.end(),
        [&cap](const Capability & c) {return c.name == cap.name;}),
      registered_capabilities_.end());
    registered_capabilities_.push_back(cap);

    if (is_spinning()) {
      register_async(cap, max_retries);
      return true;
    }
    return execute_registration(cap, timeout_sec, max_retries);
  }

  /// Notify the GroundingNode of a symbolic state change.
  ///
  /// Once the node is spinning, the update is sent asynchronously and
  /// failures are logged.
  ///
  /// @param set_tokens    State tokens to mark as True
  /// @param clear_tokens  State tokens to mark as False
  /// @param timeout_sec   How long to wait for the service
  /// @return true on success (or when sent asynchronously), false on failure (non-fatal)
  bool update_state(
    const std::vector<std::string> & set_tokens = {},
    const std::vector<std::string> & clear_tokens = {},
    double timeout_sec = 3.0)
  {
    if (set_tokens.empty() && clear_tokens.empty()) {
      return true;  // no-op
    }

    nlohmann::json j;
    j["set"] = set_tokens;
    j["clear"] = clear_tokens;

    auto request = std::make_shared<UpdateState::Request>();
    request->state_json = j.dump();

    if (is_spinning()) {
      if (!state_client_->service_is_ready()) {
        RCLCPP_WARN(
          this->get_logger(),
          "[Lingua] lingua/update_state not available — state update skipped.");
        return false;
      }
      state_client_->async_send_request(
        request,
        [this](typename rclcpp::Client<UpdateState>::SharedFuture future) {
          auto result = future.get();
          if (!result->success) {
            RCLCPP_WARN(
              this->get_logger(),
              "[Lingua] State update failed: %s", result->message.c_str());
          }
        });
      return true;
    }

    if (!state_client_->wait_for_service(
        std::chrono::duration<double>(2.0)))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "[Lingua] lingua/update_state not available — state update skipped.");
      return false;
    }

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

  /// Send lingua_current_state() to the GroundingNode.
  bool publish_current_state()
  {
    auto state = lingua_current_state();
    return update_state(state.first, state.second);
  }

protected:
  /// Override to report this node's current symbolic state as
  /// (set_tokens, clear_tokens). Sent again after every re-registration,
  /// so a restarted GroundingNode learns it too.
  virtual std::pair<std::vector<std::string>, std::vector<std::string>> lingua_current_state()
  {
    return {{}, {}};
  }

  /// Called when the GroundingNode requests re-registration.
  void handle_reregister_request()
  {
    if (registered_capabilities_.empty() || reregister_pending_) {
      return;  // nothing to do, or a re-registration is already scheduled
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[Lingua] Grounding node requested re-registration. "
      "Re-registering %zu capabilities...",
      registered_capabilities_.size());

    // Defer to a one-shot timer so this subscription callback returns immediately
    reregister_pending_ = true;
    call_later(
      std::chrono::milliseconds(100), [this]() {
        reregister_pending_ = false;
        for (const auto & cap : registered_capabilities_) {
          register_async(cap, kAsyncRegisterAttempts);
        }
        // A restarted GroundingNode has an empty state, so report ours again
        publish_current_state();
      });
  }

private:
  static constexpr int kAsyncRegisterAttempts = 3;

  /// True once the node has been added to an executor.
  bool is_spinning()
  {
    return this->get_node_base_interface()->get_associated_with_executor_atomic().load();
  }

  /// Run fn once after delay (ROS 2 timers repeat, so cancel it on first call).
  void call_later(std::chrono::nanoseconds delay, std::function<void()> fn)
  {
    auto self = std::make_shared<std::weak_ptr<rclcpp::TimerBase>>();
    auto timer = this->create_wall_timer(
      delay, [this, self, fn]() {
        if (auto t = self->lock()) {
          t->cancel();
          one_shot_timers_.erase(
            std::remove(one_shot_timers_.begin(), one_shot_timers_.end(), t),
            one_shot_timers_.end());
        }
        fn();
      });
    *self = timer;
    one_shot_timers_.push_back(timer);
  }

  /// Send a registration without blocking; retry on a timer if the service is down.
  void register_async(const Capability & cap, int attempts_left)
  {
    if (!register_client_->service_is_ready()) {
      if (attempts_left > 1) {
        RCLCPP_WARN(
          this->get_logger(),
          "[Lingua] lingua/register_capability not available for '%s'. Retrying in 1s...",
          cap.name.c_str());
        call_later(
          std::chrono::seconds(1), [this, cap, attempts_left]() {
            register_async(cap, attempts_left - 1);
          });
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "[Lingua] Could not reach lingua/register_capability to register '%s'. "
          "Is the GroundingNode running?",
          cap.name.c_str());
      }
      return;
    }

    auto request = std::make_shared<RegisterCapability::Request>();
    request->capability_json = cap.to_json();
    register_client_->async_send_request(
      request,
      [this, name = cap.name](typename rclcpp::Client<RegisterCapability>::SharedFuture future) {
        auto result = future.get();
        if (result->success) {
          RCLCPP_INFO(this->get_logger(), "Registered capability: '%s'", name.c_str());
        } else {
          RCLCPP_ERROR(
            this->get_logger(),
            "[Lingua] Registration rejected for '%s': %s",
            name.c_str(), result->message.c_str());
        }
      });
  }

  /// Internal helper: execute registration with retries + backoff (node not spinning yet).
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
  std::vector<rclcpp::TimerBase::SharedPtr> one_shot_timers_;
  bool reregister_pending_ = false;
};

}  // namespace lingua

#endif  // ROS2_LINGUA_CPP__LINGUA_MIXIN_HPP_
