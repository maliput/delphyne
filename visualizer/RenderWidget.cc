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

#include <cmath>
#include <iostream>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

#include <ignition/gui/Iface.hh>
#include <ignition/gui/Plugin.hh>

#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderTarget.hh>
#include <ignition/rendering/RenderTypes.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

namespace delphyne {
namespace gui {

/// \class RenderWidget
/// \brief This is a class that implements a simple ign-gui widget for
/// rendering a scene, using the ign-rendering functionality.
class RenderWidget : public ignition::gui::Plugin {
 public:
  /// \brief Constructor
  RenderWidget();

  /// \brief Destructor
  virtual ~RenderWidget();

 protected:
  /// \brief Overridden method to receive Qt paint event.
  /// \param[in] _e  The event that happened
  virtual void paintEvent(QPaintEvent* _e);

  /// \brief Overridden method to receive Qt show event.
  /// \param[in] _e  The event that happened
  virtual void showEvent(QShowEvent* _e);

  /// \brief Overridden method to receive Qt resize event.
  /// \param[in] _e  The event that happened.
  virtual void resizeEvent(QResizeEvent* _e);

  /// \brief Overridden method to receive Qt move event.
  /// \param[in] _e  The event that happened.
  virtual void moveEvent(QMoveEvent* _e);

  /// \brief Override paintEngine to stop Qt From trying to draw on top of
  /// render window.
  /// \return NULL.
  virtual QPaintEngine* paintEngine() const;

 private:
  /// \brief Internal method to create the render window the first time
  /// RenderWidget::showEvent is called.
  void CreateRenderWindow();

  /// \brief Pointer to timer to call update on a periodic basis.
  QTimer* updateTimer = nullptr;

  /// \brief The frequency at which we'll do an update on the widget.
  const int kUpdateTimeFrequency = static_cast<int>(std::round(1000.0 / 60.0));

  /// \brief Pointer to the renderWindow created by this class.
  ignition::rendering::RenderWindowPtr renderWindow;

  /// \brief Pointer to the camera created by this class.
  ignition::rendering::CameraPtr camera;
};

RenderWidget::RenderWidget() : Plugin() {
  this->setAttribute(Qt::WA_OpaquePaintEvent, true);
  this->setAttribute(Qt::WA_PaintOnScreen, true);
  this->setAttribute(Qt::WA_NoSystemBackground, true);

  this->setFocusPolicy(Qt::StrongFocus);
  this->setMouseTracking(true);
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // The below block means that every time the updateTime expires, we do an
  // update on the widget.  Later on, we call the start() method to start this
  // time at a fixed frequency.  Note that we do not start this timer until the
  // first time that showEvent() is called.
  this->updateTimer = new QTimer(this);
  QObject::connect(this->updateTimer, SIGNAL(timeout()), this, SLOT(update()));

  this->setMinimumHeight(100);
}

RenderWidget::~RenderWidget() {}

void RenderWidget::CreateRenderWindow() {
  std::string engineName = "ogre";
  ignition::rendering::RenderEngine* engine =
      ignition::rendering::get_engine(engineName);
  if (!engine) {
    ignerr << "Engine '" << engineName << "' is not supported" << std::endl;
    return;
  }

  // Create sample scene
  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  if (scene == nullptr) {
    ignerr << "Failed to create scene" << std::endl;
    return;
  }

  scene->SetAmbientLight(0.3, 0.3, 0.3);
  ignition::rendering::VisualPtr root = scene->RootVisual();
  if (root == nullptr) {
    ignerr << "Failed to find the root visual" << std::endl;
    return;
  }
  ignition::rendering::DirectionalLightPtr light0 =
      scene->CreateDirectionalLight();
  if (light0 == nullptr) {
    ignerr << "Failed to create a directional light" << std::endl;
    return;
  }
  light0->SetDirection(-0.5, -0.5, -1);
  light0->SetDiffuseColor(0.5, 0.5, 0.5);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  ignition::rendering::MaterialPtr green = scene->CreateMaterial();
  if (green == nullptr) {
    ignerr << "Failed to create green material" << std::endl;
    return;
  }
  green->SetAmbient(0.0, 0.5, 0.0);
  green->SetDiffuse(0.0, 0.7, 0.0);
  green->SetSpecular(0.5, 0.5, 0.5);
  green->SetShininess(50);
  green->SetReflectivity(0);

  ignition::rendering::VisualPtr vis = scene->CreateVisual();
  if (vis == nullptr) {
    ignerr << "Failed to create visual" << std::endl;
    return;
  }
  vis->AddGeometry(scene->CreateBox());
  vis->SetLocalPosition(3, 0, 0);
  vis->SetLocalScale(1, 1, 1);
  vis->SetMaterial(green);
  root->AddChild(vis);

  // create user camera
  this->camera = scene->CreateCamera("user_camera");
  if (this->camera == nullptr) {
    ignerr << "Failed to create camera" << std::endl;
    return;
  }
  this->camera->SetLocalPosition(0.0, 0.0, 0.0);
  this->camera->SetLocalRotation(0.0, 0.0, 0.0);
  this->camera->SetAspectRatio(1.333);
  this->camera->SetHFOV(M_PI / 2.0);
  root->AddChild(this->camera);

  // create render window
  std::string winHandle = std::to_string(static_cast<uint64_t>(this->winId()));
  this->renderWindow = this->camera->CreateRenderWindow();
  if (this->renderWindow == nullptr) {
    ignerr << "Failed to create camera render window" << std::endl;
    return;
  }
  this->renderWindow->SetHandle(winHandle);
  this->renderWindow->SetWidth(this->width());
  this->renderWindow->SetHeight(this->height());

  // render once to create the window.
  this->camera->Update();
}

/////////////////////////////////////////////////
void RenderWidget::showEvent(QShowEvent* _e) {
  QApplication::flush();

  if (!this->renderWindow) {
    this->CreateRenderWindow();
    this->updateTimer->start(this->kUpdateTimeFrequency);
  }

  QWidget::showEvent(_e);

  this->raise();
  this->setFocus();
}

/////////////////////////////////////////////////
QPaintEngine* RenderWidget::paintEngine() const { return nullptr; }

/////////////////////////////////////////////////
void RenderWidget::paintEvent(QPaintEvent* _e) {
  if (this->renderWindow && this->camera) {
    this->camera->Update();
  }

  _e->accept();
}

/////////////////////////////////////////////////
void RenderWidget::resizeEvent(QResizeEvent* _e) {
  if (!this->renderWindow) {
    return;
  }
  this->renderWindow->OnResize(_e->size().width(), _e->size().height());
}

/////////////////////////////////////////////////
void RenderWidget::moveEvent(QMoveEvent* _e) {
  QWidget::moveEvent(_e);

  if (!_e->isAccepted() || !this->renderWindow) {
    return;
  }
  this->renderWindow->OnMove();
}

}  // namespace gui
}  // namespace delphyne

IGN_COMMON_REGISTER_SINGLE_PLUGIN(delphyne::gui::RenderWidget,
                                  ignition::gui::Plugin);
