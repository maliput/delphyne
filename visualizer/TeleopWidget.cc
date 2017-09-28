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
#include <cmath>
#include <string>

#include "bridge/protobuf/headers/automotive_driving_command.pb.h"

#include "TeleopWidget.hh"

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

using namespace delphyne;
using namespace gui;

Q_DECLARE_METATYPE(ignition::msgs::Boolean)

/////////////////////////////////////////////////
TeleopWidget::TeleopWidget(QWidget* parent)
    : Plugin(),
      currentThrottle(0.0),
      currentBrake(0.0),
      currentSteeringAngle(0.0),
      driving(false) {
  qRegisterMetaType<ignition::msgs::Boolean>();

  this->title = "TeleopWidget";

  this->lineedit = new QLineEdit();
  this->lineedit->setText("DRIVING_COMMAND_0");

  this->button = new QPushButton("Start Driving");

  auto steeringAngleFixed = new QLabel("Steering Angle: ");
  auto throttleValueFixed = new QLabel("Throttle Value: ");
  auto brakeValueFixed = new QLabel("Brake Value: ");

  this->steeringAngleLabel = new QLabel("0.0");
  this->throttleValueLabel = new QLabel("0.0");
  this->brakeValueLabel = new QLabel("0.0");

  auto layout = new QGridLayout;
  layout->addWidget(this->lineedit, 0, 0);
  layout->addWidget(this->button, 0, 1, 1, 2);
  layout->addWidget(steeringAngleFixed, 1, 0);
  layout->addWidget(this->steeringAngleLabel, 1, 1);
  layout->addWidget(throttleValueFixed, 2, 0);
  layout->addWidget(this->throttleValueLabel, 2, 1);
  layout->addWidget(brakeValueFixed, 3, 0);
  layout->addWidget(this->brakeValueLabel, 3, 1);

  this->setLayout(layout);

  QObject::connect(this->button, SIGNAL(clicked()), this, SLOT(StartDriving()));
  QObject::connect(this, SIGNAL(RepeatingDriveTopic(const ignition::msgs::Boolean &, const bool)), this, SLOT(DriveTopicComplete(const ignition::msgs::Boolean &, const bool)));

  timer.start(10, this);
}

void TeleopWidget::LoadConfig(const tinyxml2::XMLElement* _pluginElem) {
  // Read configuration
  if (_pluginElem) {
    if (auto channelElem = _pluginElem->FirstChildElement("car_number")) {
      std::string channelName =
          "DRIVING_COMMAND_" + std::string(channelElem->GetText());
      this->lineedit->setText(QString::fromStdString(channelName));
    }
  }
}

void TeleopWidget::StartDriving() {
  if (this->driving) {
    ignmsg << "Stop Driving" << std::endl;
    this->button->setText("Start Driving");
    this->lineedit->setEnabled(true);
    this->driving = false;
    setFocus();
  } else {
    ignmsg << "Start Driving" << std::endl;
    auto lcmChannel = this->lineedit->text().toStdString();

    // Ask the bridge to start repeating this channel
    ignition::msgs::StringMsg_V request;
    request.add_data(lcmChannel);
    request.add_data("ign_msgs.AutomotiveDrivingCommand");

    this->node_.Request("/repeat_ignition_topic", request, &TeleopWidget::OnRepeatIgnitionTopic, this);
  }
}

/////////////////////////////////////////////////
void TeleopWidget::OnRepeatIgnitionTopic(const ignition::msgs::Boolean &response, const bool result) {
  emit this->RepeatingDriveTopic(response, result);
}

/////////////////////////////////////////////////
void TeleopWidget::DriveTopicComplete(const ignition::msgs::Boolean &response, const bool result) {
  auto lcmChannel = this->lineedit->text().toStdString();
  auto ignTopic = "/" + lcmChannel;

  if (!result || !response.data()) {
    ignerr << "Repeat request for " << lcmChannel << " failed" << std::endl;
    return;
  }

  this->publisher_.reset(new ignition::transport::Node::Publisher());
  *(this->publisher_) =
      this->node_.Advertise<ignition::msgs::AutomotiveDrivingCommand>(
          ignTopic);
  this->button->setText("Stop Driving");
  this->lineedit->setEnabled(false);
  this->driving = true;

  setFocus();
}

/////////////////////////////////////////////////
TeleopWidget::~TeleopWidget() {}

/////////////////////////////////////////////////
void TeleopWidget::mousePressEvent(QMouseEvent* _event) {
  setFocus();
}

/////////////////////////////////////////////////
static void sec_and_nsec_now(int64_t& sec, int32_t& nsec) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      now = std::chrono::system_clock::now();
  std::chrono::nanoseconds epoch = now.time_since_epoch();
  int64_t count = epoch.count();

  nsec = static_cast<int32_t>(count % 1000000000l);
  sec = (count - nsec) / 1000000000l;
}

/////////////////////////////////////////////////
void TeleopWidget::timerEvent(QTimerEvent* event) {
  if (event->timerId() == timer.timerId()) {
    // do our stuff
    if (this->driving) {
      double last_throttle = this->currentThrottle;
      double last_brake = this->currentBrake;

      if (!this->keepCurrentThrottle) {
        if (this->throttleKeyPressed) {
          computeClampAndSetThrottle(1.0);
        } else {
          computeClampAndSetThrottle(-6.0);
        }
      }

      if (!this->keepCurrentBrake) {
        if (this->brakeKeyPressed) {
          computeClampAndSetBrake(1.0);
        } else {
          computeClampAndSetBrake(-6.0);
        }
      }

      if (last_throttle != this->currentThrottle || last_brake != this->currentBrake ||
          this->newSteeringAngle) {
        ignition::msgs::AutomotiveDrivingCommand ignMsg;

        // We don't set the header here since the bridge completely ignores it

        int32_t nsec;
        int64_t sec;

        sec_and_nsec_now(sec, nsec);

        ignMsg.mutable_time()->set_sec(sec);
        ignMsg.mutable_time()->set_nsec(nsec);

        ignMsg.set_acceleration(this->currentThrottle - this->currentBrake);
        ignMsg.set_theta(this->currentSteeringAngle);

        this->publisher_->Publish(ignMsg);

        this->steeringAngleLabel->setText(
            QString("%1").arg(this->currentSteeringAngle));
        this->throttleValueLabel->setText(
            QString("%1").arg(this->currentThrottle));
        this->brakeValueLabel->setText(QString("%1").arg(this->currentBrake));

        this->newSteeringAngle = false;
      }
    }
  } else {
    QWidget::timerEvent(event);
  }
}

// Target velocity 60mph, i.e. ~26.8224 m/sec
static const double maxVelocity = 26.8224;
static const double throttleScale = maxVelocity / 300.0;

static const double maxBrake = maxVelocity;
static const double brakeScale = throttleScale;

// Maximum steering angle is 45 degrees (
static const double Pi = 3.1415926535897931;
static const double DegToRad = Pi / 180.0;
static const double maxSteeringAngle = 45 * DegToRad;
static const double steeringButtonStepAngle = maxSteeringAngle / 100.0;

/////////////////////////////////////////////////
void TeleopWidget::computeClampAndSetThrottle(double throttleGradient) {
  double throttle = this->currentThrottle + throttleGradient * throttleScale;
  if (throttle < 0.0) {
    throttle = 0.0;
  } else if (throttle > maxVelocity) {
    throttle = maxVelocity;
  }
  this->currentThrottle = throttle;
}

/////////////////////////////////////////////////
void TeleopWidget::computeClampAndSetBrake(double brakeGradient) {
  double brake = this->currentBrake + brakeGradient * brakeScale;
  if (brake < 0.0) {
    brake = 0.0;
  } else if (brake > maxBrake) {
    brake = maxBrake;
  }
  this->currentBrake = brake;
}

/////////////////////////////////////////////////
void TeleopWidget::computeClampAndSetSteeringAngle(double sign) {
  double angle = this->currentSteeringAngle + steeringButtonStepAngle * sign;
  if (angle > maxSteeringAngle) {
    angle = maxSteeringAngle;
  } else if (angle < -maxSteeringAngle) {
    angle = -maxSteeringAngle;
  }
  this->currentSteeringAngle = angle;
}

/////////////////////////////////////////////////
void TeleopWidget::keyPressEvent(QKeyEvent* _event) {
  if (!driving) {
    ignwarn << "Not driving, ignoring keypress" << std::endl;
    Plugin::keyPressEvent(_event);
    return;
  }

  // The list of keys is here: http://doc.qt.io/qt-5/qt.html#Key-enum
  if (_event->key() == Qt::Key_Left) {
    computeClampAndSetSteeringAngle(1.0);
    this->newSteeringAngle = true;
  } else if (_event->key() == Qt::Key_Right) {
    computeClampAndSetSteeringAngle(-1.0);
    this->newSteeringAngle = true;
  } else if (_event->key() == Qt::Key_Up) {
    this->throttleKeyPressed = true;
    this->keepCurrentThrottle = false;
  } else if (_event->key() == Qt::Key_Down) {
    this->brakeKeyPressed = true;
    this->keepCurrentBrake = false;
  } else {
    // The Qt documentation at http://doc.qt.io/qt-5/qwidget.html#keyPressEvent
    // says that you must call the base class if you don't handle the key, so
    // do that here.
    Plugin::keyPressEvent(_event);
    return;
  }
}

/////////////////////////////////////////////////
void TeleopWidget::keyReleaseEvent(QKeyEvent* _event) {
  if (!_event->isAutoRepeat()) {
    if (_event->key() == Qt::Key_Space) {
      this->keepCurrentThrottle = true;
      this->keepCurrentBrake = true;
    } else if (_event->key() == Qt::Key_Up) {
      this->throttleKeyPressed = false;
    } else if (_event->key() == Qt::Key_Down) {
      this->brakeKeyPressed = false;
    } else if (_event->key() == Qt::Key_Left || _event->key() == Qt::Key_Right) {
      // do nothing
      // this avoids the accel/brake values of not going down
      // to zero after release when steering at the same time
    } else {
      Plugin::keyReleaseEvent(_event);
    }
  }
  return;
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(delphyne::gui::TeleopWidget,
                                  ignition::gui::Plugin)
