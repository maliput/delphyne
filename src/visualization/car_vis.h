// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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

#include <string>
#include <vector>

#include <drake/lcmt_viewer_link_data.hpp>
#include <drake/systems/rendering/pose_bundle.h>

namespace delphyne {

/// CarVis is a base class that provides visualization geometries and their
/// poses.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
template <typename T>
class CarVis {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CarVis)

  /// The constructor.
  ///
  /// @param id The ID of the vehicle being visualized. This must be unique per
  /// vehicle in the same simulation.
  ///
  /// @param name The name of the vehicle being visualized. This can be any
  /// user-defined value.
  ///
  CarVis(int id, const std::string& name) : id_(id), name_(name) {}

  virtual ~CarVis() {}

  /// Returns the visualization elements.
  virtual const std::vector<drake::lcmt_viewer_link_data>& GetVisElements() const = 0;

  /// Computes and returns the poses of the bodies that constitute the vehicle's
  /// visualization. The provided `X_WM` is the pose of the vehicle model in the
  /// world frame. The origin of the model's frame is assumed to be in the
  /// middle of the vehicle's rear axle. The poses in the returned PoseBundle
  /// are for the visualization's elements, and are also in the world frame. The
  /// size of this bundle is the value returned by num_poses().
  virtual drake::systems::rendering::PoseBundle<T> CalcPoses(const drake::Isometry3<T>& X_WM) const = 0;

  /// Returns the ID that was supplied to the constructor.
  int id() const { return id_; }

  /// Returns the name that was supplied to the constructor.
  const std::string& name() const { return name_; }

  /// Returns the number of visualization geometry poses.
  int num_poses() const;

 private:
  const int id_;
  const std::string name_;
};

}  // namespace delphyne
