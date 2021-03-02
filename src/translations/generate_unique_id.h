// Copyright 2021 Toyota Research Institute

#pragma once

#include <functional>
#include <sstream>

namespace delphyne {

/// @brief Generates a unique ID for each pair of integer robot_id
/// and link name as a string. The values are serialized and concatenated
/// into a single string and a hash of that string is returned.
/// @param[in]  robot_id An integer value containing the robot index.
/// @param[in]  link_name The name of the link, which must be unique to the robot_id.
/// @return  a unique integer identifying the link.

inline size_t GenerateLinkId(size_t robot_id, const std::string& link_name) {
  std::stringstream stream;
  stream << "model[" << robot_id << "]::" << link_name;
  return std::hash<std::string>{}(stream.str());
}

/// @brief Generates a unique ID for each group of integer robot_id,
/// link name as a string, and visual id. The values are serialized and concatenated
/// into a single string and a hash of that string is returned.
/// @param[in]  robot_id An integer value containing the robot index.
/// @param[in]  link_name The name of the link, which must be unique to the robot_id.
/// @param[in]  visual_id An integer value of the visual index within the link.
/// @return  a unique integer identifying the visual.

inline size_t GenerateVisualId(size_t robot_id, const std::string& link_name, size_t visual_id) {
  std::stringstream stream;
  stream << link_name << "::visual[" << visual_id << ']';
  return GenerateLinkId(robot_id, stream.str());
}

}  // namespace delphyne
