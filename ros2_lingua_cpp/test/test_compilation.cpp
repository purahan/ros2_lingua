// Copyright 2024 ros2_lingua contributors
// SPDX-License-Identifier: Apache-2.0
//
// Compilation smoke test: verifies that LinguaMixin<rclcpp::Node> compiles
// and that a node can be instantiated with capabilities.
// Does NOT spin — just tests that the template instantiation is valid.

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "ros2_lingua_cpp/lingua_mixin.hpp"

class TestNode : public lingua::LinguaMixin<rclcpp::Node> {
public:
  TestNode()
  : LinguaMixin("test_lingua_cpp_node")
  {
    lingua::Capability cap;
    cap.name = "test_capability";
    cap.description = "A test capability for compilation check";
    cap.ros_action = "test/action";
    cap.parameters.push_back({
      "param1", "string", "A test parameter", true, ""
    });
    cap.preconditions = {"precond_a"};
    cap.postconditions = {"postcond_b"};
    cap.tags = {lingua::Tags::LOCOMOTION};

    // Store the capability JSON for verification
    test_json_ = cap.to_json();
  }

  std::string get_test_json() const { return test_json_; }

private:
  std::string test_json_;
};

TEST(CompilationTest, NodeInstantiation) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<TestNode>();

  // Verify the node was created and has the right name
  EXPECT_EQ(node->get_name(), std::string("test_lingua_cpp_node"));

  // Verify the capability JSON was generated
  EXPECT_FALSE(node->get_test_json().empty());

  // Parse the JSON to verify structure
  auto j = nlohmann::json::parse(node->get_test_json());
  EXPECT_EQ(j["name"], "test_capability");
  EXPECT_EQ(j["ros_action"], "test/action");

  node.reset();
  rclcpp::shutdown();
}
