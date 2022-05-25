// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2021-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
