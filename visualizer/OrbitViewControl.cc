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

#include <mutex>

#include <ignition/common/Console.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/rendering/Camera.hh>

#include "OrbitViewControl.hh"

using namespace delphyne;
using namespace gui;

/////////////////////////////////////////////////
OrbitViewControl::OrbitViewControl(const ignition::rendering::CameraPtr& _cam)
    : camera(_cam) {
  this->rayQuery = this->camera->Scene()->CreateRayQuery();
  if (!this->rayQuery) {
    ignerr << "Failed to create Ray Query" << std::endl;
    return;
  }
}

/////////////////////////////////////////////////
OrbitViewControl::~OrbitViewControl() {}

/////////////////////////////////////////////////
void OrbitViewControl::OnMousePress(const QMouseEvent* _e) {
  this->OnMouseButtonAction(_e, ButtonState_t::PRESSED);
}

/////////////////////////////////////////////////
void OrbitViewControl::OnMouseRelease(const QMouseEvent* _e) {
  this->OnMouseButtonAction(_e, ButtonState_t::RELEASED);
}

/////////////////////////////////////////////////
void OrbitViewControl::OnMouseMove(const QMouseEvent* _e) {
  if (!this->camera) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->mouseMutex);
    int deltaX = _e->x() - this->mouse.motionX;
    int deltaY = _e->y() - this->mouse.motionY;
    this->mouse.motionX = _e->x();
    this->mouse.motionY = _e->y();

    if (this->mouse.motionDirty) {
      this->mouse.dragX += deltaX;
      this->mouse.dragY += deltaY;
    } else {
      this->mouse.dragX = deltaX;
      this->mouse.dragY = deltaY;
    }
    this->mouse.motionDirty = true;
  }

  this->Update();
}

/////////////////////////////////////////////////
void OrbitViewControl::OnMouseWheel(const QWheelEvent* _e) {
  if (!this->camera) {
    return;
  }

  std::lock_guard<std::mutex> lock(this->mouseMutex);
  double scroll = (_e->angleDelta().y() > 0) ? -1.0 : 1.0;
  double distance = this->camera->WorldPosition().Distance(this->target.point);
  int factor = 1;
  double amount = -(scroll * factor) * (distance / 5.0);

  this->viewControl.SetCamera(this->camera);
  this->viewControl.SetTarget(this->target.point);
  this->viewControl.Zoom(amount);
}

//////////////////////////////////////////////////
void OrbitViewControl::OnMouseButtonAction(const QMouseEvent* _e,
                                           const ButtonState_t& _state) {
  if (!this->camera) {
    return;
  }

  // Ignore unknown mouse buttons.
  if (_e->button() != Qt::LeftButton && _e->button() != Qt::RightButton &&
      _e->button() != Qt::MidButton) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->mouseMutex);
    this->mouse.button = _e->button();
    this->mouse.state = _state;
    this->mouse.x = _e->x();
    this->mouse.y = _e->y();
    this->mouse.motionX = _e->x();
    this->mouse.motionY = _e->y();
    this->mouse.buttonDirty = true;
  }

  this->Update();
}

//////////////////////////////////////////////////
void OrbitViewControl::Update() {
  if (!this->camera || !this->rayQuery) {
    return;
  }

  std::lock_guard<std::mutex> lock(this->mouseMutex);
  if (this->mouse.buttonDirty) {
    this->mouse.buttonDirty = false;
    double nx =
        2.0 * this->mouse.x / static_cast<double>(this->camera->ImageWidth()) -
        1.0;
    double ny =
        1.0 -
        2.0 * this->mouse.y / static_cast<double>(this->camera->ImageHeight());
    this->rayQuery->SetFromCamera(this->camera,
                                  ignition::math::Vector2d(nx, ny));
    this->target = this->rayQuery->ClosestPoint();
    if (!this->target) {
      this->target.point =
          this->rayQuery->Origin() + this->rayQuery->Direction() * 10;
      return;
    }
  }

  if (this->mouse.motionDirty) {
    this->mouse.motionDirty = false;
    auto drag = ignition::math::Vector2d(this->mouse.dragX, this->mouse.dragY);

    // Left mouse button pan.
    if (this->mouse.button == Qt::LeftButton &&
        this->mouse.state == ButtonState_t::PRESSED) {
      this->viewControl.SetCamera(this->camera);
      this->viewControl.SetTarget(this->target.point);
      this->viewControl.Pan(drag);
    } else if (this->mouse.button == Qt::MidButton &&
               this->mouse.state == ButtonState_t::PRESSED) {
      this->viewControl.SetCamera(this->camera);
      this->viewControl.SetTarget(this->target.point);
      this->viewControl.Orbit(drag);
    }
    // Right mouse button zoom.
    else if (this->mouse.button == Qt::RightButton &&
             this->mouse.state == ButtonState_t::PRESSED) {
      double hfov = this->camera->HFOV().Radian();
      double vfov = 2.0f * atan(tan(hfov / 2.0f) / this->camera->AspectRatio());
      double distance =
          this->camera->WorldPosition().Distance(this->target.point);
      double amount = ((-this->mouse.dragY /
                        static_cast<double>(this->camera->ImageHeight())) *
                       distance * tan(vfov / 2.0) * 6.0);

      this->viewControl.SetCamera(this->camera);
      this->viewControl.SetTarget(this->target.point);
      this->viewControl.Zoom(amount);
    }
  }
}
