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

}  // namespace delphyne
