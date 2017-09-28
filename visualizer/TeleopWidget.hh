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

#ifndef DELPHYNE_GUI_TELEOPWIDGET_HH
#define DELPHYNE_GUI_TELEOPWIDGET_HH

#include <memory>

#include <ignition/gui/Plugin.hh>
#include <ignition/transport.hh>

namespace delphyne {
namespace gui {

/// \class TeleopWidget
/// \brief This is a class that implements a simple ign-gui widget for
/// teleop-ing.
class TeleopWidget : public ignition::gui::Plugin {
  Q_OBJECT

 public:
  /// \brief Default constructor.
  explicit TeleopWidget(QWidget* parent = 0);

  /// \brief Default Destructor.
  virtual ~TeleopWidget();

  public slots: void StartDriving();

  signals: void RepeatingDriveTopic(const ignition::msgs::Boolean &response, const bool result);

  public slots: void DriveTopicComplete(const ignition::msgs::Boolean &response, const bool result);

 protected:
  virtual void keyPressEvent(QKeyEvent* _event) override;
  virtual void keyReleaseEvent(QKeyEvent* _event) override;
  void mousePressEvent(QMouseEvent* _event) override;
  void timerEvent(QTimerEvent* event) override;
  void LoadConfig(const tinyxml2::XMLElement* _pluginElem) override;
  void OnRepeatIgnitionTopic(const ignition::msgs::Boolean &response, const bool result);

 private:
  /// \internal
  /// \brief A transport node.
  ignition::transport::Node node_;

  /// \internal
  /// \brief The ignition publisher used to send the updates
  std::unique_ptr<ignition::transport::Node::Publisher> publisher_;

  /// \internal
  /// \brief The current amount of throttle
  double currentThrottle;

  /// \internal
  /// \brief The current amount of brake
  double currentBrake;

  /// \internal
  /// \brief True if any valid key is pressed
  bool keyIsPressed = false;

  /// \internal
  /// \brief True if keeping current throttle or
  /// brake value without reseting to zero is enabled.
  bool keepCurrentThrottleBrake = false;

  /// \internal
  /// \brief The current steering angle
  double currentSteeringAngle;

  /// \internal
  /// \brief The current state of the widget
  bool driving;

  QLineEdit* lineedit;
  QPushButton* button;
  QLabel* steeringAngleLabel;
  QLabel* throttleValueLabel;
  QLabel* brakeValueLabel;
  QBasicTimer timer;

  /// \internal
  /// \brief Compute and set a valid throttle value
  void computeClampAndSetThrottle(double throttleGradient);

  /// \internal
  /// \brief Compute and set a valid brake value
  void computeClampAndSetBrake(double brakeGradient);

  /// \internal
  /// \brief Compute and set a valid steering angle value
  void computeClampAndSetSteeringAngle(double sign, double step);
};
}
}

#endif
