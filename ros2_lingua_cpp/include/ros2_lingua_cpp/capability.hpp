// Copyright 2024 ros2_lingua contributors
// SPDX-License-Identifier: Apache-2.0
//
// ros2_lingua_cpp/capability.hpp
// --------------------------------
// C++ equivalents of Python's Capability and CapabilityParameter.
// Produces identical JSON to ros2_lingua_core.schema.Capability.to_json().

#ifndef ROS2_LINGUA_CPP__CAPABILITY_HPP_
#define ROS2_LINGUA_CPP__CAPABILITY_HPP_

#include <string>
#include <vector>
#include <map>
#include <optional>

#include "ros2_lingua_cpp/json.hpp"

namespace lingua {

/// Describes a single input parameter for a capability.
struct CapabilityParameter {
  std::string name;
  std::string type;         // "string" | "float" | "int" | "bool" | etc.
  std::string description;
  bool required = true;
  std::string default_value;  // empty string if none

  nlohmann::json to_json() const {
    nlohmann::json j;
    j["name"] = name;
    j["type"] = type;
    j["description"] = description;
    j["required"] = required;
    if (!default_value.empty()) {
      j["default"] = default_value;
    } else {
      j["default"] = nullptr;
    }
    return j;
  }
};

/// Describes one thing a ROS 2 node can do.
///
/// This is the C++ equivalent of ros2_lingua_core.schema.Capability.
/// The JSON output is wire-compatible with the Python version.
struct Capability {
  // --- Identity ---
  std::string name;
  std::string description;

  // --- ROS 2 Interface (exactly one should be set) ---
  std::string ros_action;   // empty if using service
  std::string ros_service;  // empty if using action

  // --- Parameters ---
  std::vector<CapabilityParameter> parameters;

  // --- State conditions (used for chaining) ---
  std::vector<std::string> preconditions;
  std::vector<std::string> postconditions;

  // --- Optional metadata ---
  std::map<std::string, std::string> metadata;

  // --- Tags ---
  std::vector<std::string> tags;

  /// Serialise to a JSON string identical to the Python Capability.to_json().
  std::string to_json() const {
    nlohmann::json j;
    j["name"] = name;
    j["description"] = description;
    j["ros_action"] = ros_action.empty() ? nlohmann::json(nullptr) : nlohmann::json(ros_action);
    j["ros_service"] = ros_service.empty() ? nlohmann::json(nullptr) : nlohmann::json(ros_service);

    nlohmann::json params_arr = nlohmann::json::array();
    for (const auto & p : parameters) {
      params_arr.push_back(p.to_json());
    }
    j["parameters"] = params_arr;

    j["preconditions"] = preconditions;
    j["postconditions"] = postconditions;

    nlohmann::json meta_obj = nlohmann::json::object();
    for (const auto & kv : metadata) {
      meta_obj[kv.first] = kv.second;
    }
    j["metadata"] = meta_obj;

    j["tags"] = tags;

    return j.dump(2);
  }
};

// --- Standard tag constants (mirrors Python Tags class) ---
namespace Tags {
  constexpr const char * LOCOMOTION   = "locomotion";
  constexpr const char * MANIPULATION = "manipulation";
  constexpr const char * BALANCE      = "balance";
  constexpr const char * PERCEPTION   = "perception";
  constexpr const char * MAPPING      = "mapping";
  constexpr const char * SPEECH       = "speech";
  constexpr const char * SOCIAL       = "social";
  constexpr const char * SYSTEM       = "system";
  constexpr const char * SAFETY       = "safety";
  constexpr const char * NAVIGATION   = "navigation";
  constexpr const char * INSPECTION   = "inspection";
}  // namespace Tags

}  // namespace lingua

#endif  // ROS2_LINGUA_CPP__CAPABILITY_HPP_
