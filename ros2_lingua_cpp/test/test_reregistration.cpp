// Copyright 2024 ros2_lingua contributors
// SPDX-License-Identifier: Apache-2.0
//
// Re-registration while spinning (issue #29).
//
// A fake GroundingNode serves lingua/register_capability and
// lingua/update_state and publishes lingua/request_reregister. The node
// under test uses LinguaMixin and is spinning in an executor when the
// re-register request arrives, which used to throw "Node has already been
// added to an executor" and abort the process.
//
// Both nodes live in their own namespace so other tests running at the
// same time can't see them.

#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "ros2_lingua_cpp/lingua_mixin.hpp"
#include "ros2_lingua_interfaces/srv/register_capability.hpp"
#include "ros2_lingua_interfaces/srv/update_state.hpp"
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;

namespace
{

const char * kNamespace = "/test_cpp_reregistration";

/// Stand-in for the GroundingNode: counts registrations and state updates.
class FakeGroundingNode : public rclcpp::Node
{
public:
  FakeGroundingNode()
  : rclcpp::Node("fake_grounding_node", kNamespace)
  {
    register_srv_ = create_service<ros2_lingua_interfaces::srv::RegisterCapability>(
      "lingua/register_capability",
      [this](
        const std::shared_ptr<ros2_lingua_interfaces::srv::RegisterCapability::Request> req,
        std::shared_ptr<ros2_lingua_interfaces::srv::RegisterCapability::Response> res) {
        auto name = nlohmann::json::parse(req->capability_json)["name"].get<std::string>();
        std::lock_guard<std::mutex> lock(mutex_);
        registrations_[name]++;
        res->success = true;
      });
    state_srv_ = create_service<ros2_lingua_interfaces::srv::UpdateState>(
      "lingua/update_state",
      [this](
        const std::shared_ptr<ros2_lingua_interfaces::srv::UpdateState::Request> req,
        std::shared_ptr<ros2_lingua_interfaces::srv::UpdateState::Response> res) {
        auto state = nlohmann::json::parse(req->state_json);
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & token : state["set"]) {
          state_set_.push_back(token.get<std::string>());
        }
        res->success = true;
      });
    reregister_pub_ = create_publisher<std_msgs::msg::Empty>("lingua/request_reregister", 10);
  }

  int registrations(const std::string & name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return registrations_[name];
  }

  int times_state_set(const std::string & token)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    int count = 0;
    for (const auto & t : state_set_) {
      count += (t == token) ? 1 : 0;
    }
    return count;
  }

  size_t reregister_subscribers() {return reregister_pub_->get_subscription_count();}
  void request_reregister() {reregister_pub_->publish(std_msgs::msg::Empty());}

private:
  rclcpp::Service<ros2_lingua_interfaces::srv::RegisterCapability>::SharedPtr register_srv_;
  rclcpp::Service<ros2_lingua_interfaces::srv::UpdateState>::SharedPtr state_srv_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reregister_pub_;
  std::mutex mutex_;
  std::map<std::string, int> registrations_;
  std::vector<std::string> state_set_;
};

/// A robot node with two capabilities that reports one state token.
class RobotNode : public lingua::LinguaMixin<rclcpp::Node>
{
public:
  RobotNode()
  : LinguaMixin("cpp_robot_node", kNamespace)
  {
    for (const char * name : {"cpp_move", "cpp_grip"}) {
      lingua::Capability cap;
      cap.name = name;
      cap.description = "Test capability";
      cap.ros_action = "cpp_robot_node/lingua";
      cap.tags = {"test"};
      register_capability(cap, 5.0, 1);
    }
  }

protected:
  std::pair<std::vector<std::string>, std::vector<std::string>> lingua_current_state() override
  {
    return {{"cpp_ready"}, {}};
  }
};

/// Wait until predicate() is true or timeout expires.
template<typename Predicate>
bool wait_until(Predicate predicate, std::chrono::milliseconds timeout)
{
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return predicate();
}

}  // namespace

TEST(ReregistrationTest, ReregistersOncePerRequestWhileSpinning)
{
  rclcpp::init(0, nullptr);

  // The fake grounding node spins on its own, so the robot's blocking
  // registration in its constructor gets an answer.
  auto grounding = std::make_shared<FakeGroundingNode>();
  rclcpp::executors::SingleThreadedExecutor grounding_executor;
  grounding_executor.add_node(grounding);
  std::thread grounding_thread([&grounding_executor]() {grounding_executor.spin();});

  auto robot = std::make_shared<RobotNode>();
  EXPECT_EQ(grounding->registrations("cpp_move"), 1);
  EXPECT_EQ(grounding->registrations("cpp_grip"), 1);

  // Now the robot is spinning, as it would be in a real node
  rclcpp::executors::MultiThreadedExecutor robot_executor;
  robot_executor.add_node(robot);
  std::thread robot_thread([&robot_executor]() {robot_executor.spin();});

  ASSERT_TRUE(wait_until([&]() {return grounding->reregister_subscribers() > 0;}, 5000ms));

  // One request: each capability registers exactly once more, and the
  // node reports its state again
  grounding->request_reregister();
  EXPECT_TRUE(wait_until([&]() {return grounding->registrations("cpp_move") == 2;}, 5000ms));
  EXPECT_TRUE(wait_until([&]() {return grounding->registrations("cpp_grip") == 2;}, 5000ms));
  EXPECT_TRUE(wait_until([&]() {return grounding->times_state_set("cpp_ready") == 1;}, 5000ms));

  // No flood: nothing more arrives without another request
  std::this_thread::sleep_for(1s);
  EXPECT_EQ(grounding->registrations("cpp_move"), 2);
  EXPECT_EQ(grounding->registrations("cpp_grip"), 2);

  // A second request works the same way
  grounding->request_reregister();
  EXPECT_TRUE(wait_until([&]() {return grounding->registrations("cpp_move") == 3;}, 5000ms));
  EXPECT_TRUE(wait_until([&]() {return grounding->registrations("cpp_grip") == 3;}, 5000ms));

  // update_state() from outside a callback while spinning must not throw either
  EXPECT_TRUE(robot->update_state({"cpp_extra"}, {}));
  EXPECT_TRUE(wait_until([&]() {return grounding->times_state_set("cpp_extra") == 1;}, 5000ms));

  robot_executor.cancel();
  grounding_executor.cancel();
  robot_thread.join();
  grounding_thread.join();
  robot.reset();
  grounding.reset();
  rclcpp::shutdown();
}
