// Copyright 2017 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef DELPHYNE_GUI_ORBITVIEWCONTROL_HH
#define DELPHYNE_GUI_ORBITVIEWCONTROL_HH

#include <mutex>

#include <ignition/gui/Plugin.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/OrbitViewController.hh>
#include <ignition/rendering/RayQuery.hh>

namespace delphyne {
namespace gui {

/// \def ButtonState_t Possible button states.
enum class ButtonState_t {
  /// \brief Button has been pressed.
  PRESSED,
  /// \brief Button has been released.
  RELEASED,
};

/// \brief Stores the mouse state.
struct MouseButton {
  /// \brief The active button.
  Qt::MouseButton button = Qt::NoButton;
  /// \brief Pressed or released.
  ButtonState_t state = ButtonState_t::RELEASED;
  /// \brief X position of the cursor relative to the widget that received the
  /// event after a press or release.
  int x = 0;
  /// \brief Y position of the cursor relative to the widget that received the
  /// event after a press or release.
  int y = 0;
  /// \brief X position of the cursor relative to the widget that received the
  /// event after a move.
  int motionX = 0;
  /// \brief Y position of the cursor relative to the widget that received the
  /// event after a move.
  int motionY = 0;
  /// \brief Number of pixels in X position during dragging.
  int dragX = 0;
  /// \brief Number of pixels in Y position during dragging.
  int dragY = 0;
  /// \bried When true, a button action needs to be processed.
  bool buttonDirty = false;
  /// \brief When true, a move action needs to be processed.
  bool motionDirty = false;
};

/// \class OrbitViewControl
/// \brief Modifies the camera view according to the mouse events received.
/// Note that the mouse events need to be captured outside of this class.
class OrbitViewControl {
 public:
  /// \brief Default constructor.
  /// \param[in] _cam The camera to be controlled by this class.
  explicit OrbitViewControl(const ignition::rendering::CameraPtr& _cam);

  /// \brief Default Destructor.
  virtual ~OrbitViewControl();

  /// \brief Callback executed after a mouse press event.
  /// \param[in] _e The Qt mouse event.
  void OnMousePress(const QMouseEvent* _e);

  /// \brief Callback executed after a mouse release event.
  /// \param[in] _e The Qt mouse event.
  void OnMouseRelease(const QMouseEvent* _e);

  /// \brief Callback executed after a mouse move event.
  /// \param[in] _e The Qt mouse event.
  void OnMouseMove(const QMouseEvent* _e);

  /// \brief Callback executed after a mouse wheel event.
  /// \param[in] _e The Qt mouse event.
  void OnMouseWheel(const QWheelEvent* _e);

 private:
  /// \brief Update the camera view.
  void Update();

  /// \brief Callback executed after a mouse click event.
  /// \param[in] _e The Qt mouse event.
  /// \param[in] _state The state of the button.
  void OnMouseButtonAction(const QMouseEvent* _e, const ButtonState_t& _state);

  /// \brief Allows to pan, rotate and zoom the camera view.
  ignition::rendering::OrbitViewController viewControl;

  /// \brief The camera to control.
  ignition::rendering::CameraPtr camera;

  /// \brief Let us cast some rays.
  ignition::rendering::RayQueryPtr rayQuery;

  /// \brief Stores the result of a ray casting.
  ignition::rendering::RayQueryResult target;

  /// \brief Stores the mouse state.
  struct MouseButton mouse;

  /// \brief Protect the MouseButton struct from being simultaneously modified
  /// and read.
  std::mutex mouseMutex;
};
}
}

#endif
