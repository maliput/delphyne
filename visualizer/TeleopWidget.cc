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

#include <chrono>

#include "protobuf/headers/automotive_driving_command.pb.h"

#include "TeleopWidget.hh"

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

using namespace delphyne;
using namespace gui;

/////////////////////////////////////////////////
TeleopWidget::TeleopWidget(QWidget *parent)
  : Plugin(), current_throttle(0.0), current_brake(0.0), current_steering_angle(0.0), current_model_index(0), driving(false)
{
  this->title = "TeleopWidget";

  this->combobox = new QComboBox();
  this->combobox->addItem("Item 1");
  this->combobox->addItem("Item 2");

  this->button = new QPushButton("Start Driving");

  auto steering_angle_fixed = new QLabel("Steering Angle: ");
  auto throttle_value_fixed = new QLabel("Throttle Value: ");
  auto brake_value_fixed = new QLabel("Brake Value: ");

  this->steering_angle_label = new QLabel("0.0");
  this->throttle_value_label = new QLabel("0.0");
  this->brake_value_label = new QLabel("0.0");

  auto layout = new QGridLayout;
  layout->addWidget(this->combobox, 0, 0);
  layout->addWidget(this->button, 0, 1, 1, 2);
  layout->addWidget(steering_angle_fixed, 1, 0);
  layout->addWidget(this->steering_angle_label, 1, 1);
  layout->addWidget(throttle_value_fixed, 2, 0);
  layout->addWidget(this->throttle_value_label, 2, 1);
  layout->addWidget(brake_value_fixed, 3, 0);
  layout->addWidget(this->brake_value_label, 3, 1);

  this->setLayout(layout);

  QObject::connect(this->combobox, SIGNAL(currentIndexChanged(int)), this, SLOT(selectModel(int)));

  QObject::connect(this->button, SIGNAL(clicked()), this, SLOT(startDriving()));
}

void TeleopWidget::selectModel(int index)
{
  ignerr << "selectModel" << std::endl;
  ignerr << this->combobox->itemText(index).toStdString() << std::endl;
  this->current_model_index = index;
}

void TeleopWidget::startDriving()
{
  if (this->driving) {
    ignerr << "Stop Driving" << std::endl;
    this->button->setText("Start Driving");
    this->combobox->setEnabled(true);
    this->driving = false;
  }
  else {
    ignerr << "Start Driving" << std::endl;
    this->publisher_.reset(new ignition::transport::Node::Publisher());
    *(this->publisher_) = this->node_.Advertise<ignition::msgs::AutomotiveDrivingCommand>("/DRIVING_COMMAND_0");
    this->button->setText("Stop Driving");
    this->combobox->setEnabled(false);
    this->driving = true;
  }
  setFocus();
}

/////////////////////////////////////////////////
TeleopWidget::~TeleopWidget()
{
}

/////////////////////////////////////////////////
void TeleopWidget::mousePressEvent(QMouseEvent *_event)
{
  ignerr << "Mouse press!" << std::endl;
  setFocus();
}

/////////////////////////////////////////////////
static void sec_and_nsec_now(int64_t &sec, int32_t &nsec)
{
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> now = std::chrono::system_clock::now();
  std::chrono::nanoseconds epoch = now.time_since_epoch();
  int64_t count = epoch.count();

  nsec = static_cast<int32_t>(count % 1000000000l);
  sec = (count - nsec) / 1000000000l;
}

// Target velocity 60mph, i.e. ~26.8224 m/sec
static const double max_velocity = 26.8224;
static const double throttle_scale = max_velocity / 300.0;

static const double max_brake = max_velocity;
static const double brake_scale = throttle_scale;

// Maximum steering angle is 45 degrees (
static const double Pi       = 3.1415926535897931;
static const double DegToRad = Pi / 180.0;
static const double max_steering_angle = 45 * DegToRad;
static const double steering_button_step_angle = max_steering_angle / 100.0;

/////////////////////////////////////////////////
void TeleopWidget::computeClampAndSetThrottle(double throttle_gradient)
{
  double throttle = this->current_throttle + throttle_gradient * throttle_scale;
  if (throttle < 0.0) {
    throttle = 0.0;
  }
  else if (throttle > max_velocity) {
    throttle = max_velocity;
  }
  this->current_throttle = throttle;
}

/////////////////////////////////////////////////
void TeleopWidget::computeClampAndSetBrake(double brake_gradient)
{
  double brake = this->current_brake + brake_gradient * brake_scale;
  if (brake < 0.0) {
    brake = 0.0;
  }
  else if (brake > max_brake) {
    brake = max_brake;
  }
  this->current_brake = brake;
}

/////////////////////////////////////////////////
void TeleopWidget::computeClampAndSetSteeringAngle(double sign, double step)
{
  double angle = this->current_steering_angle + step * sign;
  if (angle > max_steering_angle) {
    angle = max_steering_angle;
  }
  else if (angle < -max_steering_angle) {
    angle = -max_steering_angle;
  }

  this->current_steering_angle = angle;
}

/////////////////////////////////////////////////
void TeleopWidget::keyPressEvent(QKeyEvent *_event)
{
  if (!driving) {
    ignerr << "Not driving, ignoring keypress" << std::endl;
    return;
  }

  double throttle_gradient = 0.0;
  double brake_gradient = 0.0;
  double steering_sign = 1.0;
  double steering_step = 0.0;

  // The list of keys is here: http://doc.qt.io/qt-5/qt.html#Key-enum
  if (_event->key() == Qt::Key_Left) {
    ignerr << "Left" << std::endl;
    steering_sign = 1.0;
    steering_step = steering_button_step_angle;
  }
  else if (_event->key() == Qt::Key_Right) {
    ignerr << "Right" << std::endl;
    steering_sign = -1.0;
    steering_step = steering_button_step_angle;
  }
  else if (_event->key() == Qt::Key_Up) {
    ignerr << "Up" << std::endl;
    throttle_gradient = 1.0;
  }
  else if (_event->key() == Qt::Key_Down) {
    ignerr << "Down" << std::endl;
    brake_gradient = 1.0;
  }
  else {
    // The Qt documentation at http://doc.qt.io/qt-5/qwidget.html#keyPressEvent
    // says that you must call the base class if you don't handle the key, so
    // do that here.
    Plugin::keyPressEvent(_event);
    return;
  }

  ignition::msgs::AutomotiveDrivingCommand ignMsg;

  // We don't set the header here since the bridge completely ignores it

  int32_t nsec;
  int64_t sec;

  sec_and_nsec_now(sec, nsec);

  computeClampAndSetThrottle(throttle_gradient);
  computeClampAndSetBrake(brake_gradient);
  computeClampAndSetSteeringAngle(steering_sign, steering_step);

  ignMsg.mutable_time()->set_sec(sec);
  ignMsg.mutable_time()->set_nsec(nsec);

  ignMsg.set_acceleration(this->current_throttle - this->current_brake);
  ignMsg.set_theta(this->current_steering_angle);

  ignerr << "Publish accel " << this->current_throttle - this->current_brake << ", theta " << this->current_steering_angle << std::endl;
  this->publisher_->Publish(ignMsg);

  this->steering_angle_label->setText(QString("%1").arg(this->current_steering_angle));
  this->throttle_value_label->setText(QString("%1").arg(this->current_throttle));
  this->brake_value_label->setText(QString("%1").arg(this->current_brake));
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(delphyne::gui::TeleopWidget,
                                  ignition::gui::Plugin)
