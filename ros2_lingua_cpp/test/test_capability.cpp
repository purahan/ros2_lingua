// Copyright 2024 ros2_lingua contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include "ros2_lingua_cpp/capability.hpp"
#include "ros2_lingua_cpp/json.hpp"

TEST(CapabilityTest, BasicToJson) {
  lingua::Capability cap;
  cap.name = "navigate_to_location";
  cap.description = "Moves the robot to a named location";
  cap.ros_action = "robot/navigate";
  cap.parameters.push_back({
    "location_name", "string", "Where to go", true, ""
  });
  cap.preconditions = {"robot_is_balanced"};
  cap.postconditions = {"robot_at_location"};
  cap.tags = {"locomotion", "navigation"};

  std::string json_str = cap.to_json();
  auto j = nlohmann::json::parse(json_str);

  EXPECT_EQ(j["name"], "navigate_to_location");
  EXPECT_EQ(j["description"], "Moves the robot to a named location");
  EXPECT_EQ(j["ros_action"], "robot/navigate");
  EXPECT_TRUE(j["ros_service"].is_null());
  EXPECT_EQ(j["preconditions"].size(), 1u);
  EXPECT_EQ(j["preconditions"][0], "robot_is_balanced");
  EXPECT_EQ(j["postconditions"].size(), 1u);
  EXPECT_EQ(j["postconditions"][0], "robot_at_location");
  EXPECT_EQ(j["tags"].size(), 2u);
  EXPECT_EQ(j["tags"][0], "locomotion");
  EXPECT_EQ(j["tags"][1], "navigation");
}

TEST(CapabilityTest, ParameterToJson) {
  lingua::CapabilityParameter p;
  p.name = "speed";
  p.type = "float";
  p.description = "Movement speed";
  p.required = false;
  p.default_value = "0.5";

  auto j = p.to_json();

  EXPECT_EQ(j["name"], "speed");
  EXPECT_EQ(j["type"], "float");
  EXPECT_EQ(j["description"], "Movement speed");
  EXPECT_EQ(j["required"], false);
  EXPECT_EQ(j["default"], "0.5");
}

TEST(CapabilityTest, ParameterDefaultNull) {
  lingua::CapabilityParameter p;
  p.name = "target";
  p.type = "string";
  p.description = "Target object";
  p.required = true;
  // default_value is empty string → should become null in JSON

  auto j = p.to_json();

  EXPECT_TRUE(j["default"].is_null());
}

TEST(CapabilityTest, ServiceCapability) {
  lingua::Capability cap;
  cap.name = "say";
  cap.description = "Speaks a message aloud";
  cap.ros_service = "robot/tts";
  cap.parameters.push_back({
    "message", "string", "What to say", true, ""
  });
  cap.tags = {"speech"};

  std::string json_str = cap.to_json();
  auto j = nlohmann::json::parse(json_str);

  EXPECT_TRUE(j["ros_action"].is_null());
  EXPECT_EQ(j["ros_service"], "robot/tts");
}

TEST(CapabilityTest, MetadataRoundTrip) {
  lingua::Capability cap;
  cap.name = "pick_up";
  cap.description = "Picks up an object";
  cap.ros_action = "robot/pick";
  cap.metadata["body_part"] = "left_arm";
  cap.metadata["max_payload_kg"] = "1.5";

  std::string json_str = cap.to_json();
  auto j = nlohmann::json::parse(json_str);

  EXPECT_EQ(j["metadata"]["body_part"], "left_arm");
  EXPECT_EQ(j["metadata"]["max_payload_kg"], "1.5");
}

TEST(CapabilityTest, EmptyCapability) {
  lingua::Capability cap;
  cap.name = "minimal";
  cap.description = "A minimal capability";
  cap.ros_action = "robot/minimal";

  std::string json_str = cap.to_json();
  auto j = nlohmann::json::parse(json_str);

  EXPECT_EQ(j["parameters"].size(), 0u);
  EXPECT_EQ(j["preconditions"].size(), 0u);
  EXPECT_EQ(j["postconditions"].size(), 0u);
  EXPECT_EQ(j["tags"].size(), 0u);
  EXPECT_TRUE(j["metadata"].is_object());
  EXPECT_TRUE(j["metadata"].empty());
}

TEST(TagsTest, Constants) {
  EXPECT_STREQ(lingua::Tags::LOCOMOTION, "locomotion");
  EXPECT_STREQ(lingua::Tags::MANIPULATION, "manipulation");
  EXPECT_STREQ(lingua::Tags::BALANCE, "balance");
  EXPECT_STREQ(lingua::Tags::PERCEPTION, "perception");
  EXPECT_STREQ(lingua::Tags::SPEECH, "speech");
  EXPECT_STREQ(lingua::Tags::SAFETY, "safety");
  EXPECT_STREQ(lingua::Tags::NAVIGATION, "navigation");
}
