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

#include <cstdlib>
#include <functional>
#include <map>
#include <string>
#include <utility>

#include <ignition/common/Console.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/gui/Iface.hh>
#include <ignition/gui/Plugin.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/Light.hh>
#include <ignition/rendering/MeshDescriptor.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderEngineManager.hh>
#include <ignition/rendering/RenderTarget.hh>
#include <ignition/rendering/RenderTypes.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include "RenderWidget.hh"

Q_DECLARE_METATYPE(ignition::msgs::Model)
Q_DECLARE_METATYPE(ignition::msgs::PosesStamped)

using namespace delphyne;
using namespace gui;

/////////////////////////////////////////////////
static void setPoseFromMessage(const ignition::msgs::Visual &_vis,
  ignition::rendering::VisualPtr _shape)
{
  double x = 2.0;
  double y = 0.0;
  double z = 0.0;
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;

  if (_vis.has_pose()) {
    if (_vis.pose().has_position()) {
      // The default Ogre coordinate system is X left/right, Y up/down,
      // and Z in/out (of the screen).  However, ignition-rendering switches that
      // to be consistent with Gazebo.  Thus, the coordinate system is X in/out,
      // Y left/right, and Z up/down.
      x += _vis.pose().position().z();
      y += -_vis.pose().position().x();
      z += _vis.pose().position().y();
    }
    if (_vis.pose().has_orientation()) {
      qw = _vis.pose().orientation().w();
      qx = _vis.pose().orientation().x();
      qy = _vis.pose().orientation().y();
      qz = _vis.pose().orientation().z();
    }
  }

  ignition::math::Pose3d newpose(x, y, z, qw, qx, qy, qz);

  _shape->SetLocalPose(newpose);
}

/////////////////////////////////////////////////
RenderWidget::RenderWidget(QWidget *parent)
  : Plugin(), initializedScene(false)
{
  qRegisterMetaType<ignition::msgs::Model>();
  qRegisterMetaType<ignition::msgs::PosesStamped>();

  this->setAttribute(Qt::WA_OpaquePaintEvent, true);
  this->setAttribute(Qt::WA_PaintOnScreen, true);
  this->setAttribute(Qt::WA_NoSystemBackground, true);

  this->setFocusPolicy(Qt::StrongFocus);
  this->setMouseTracking(true);
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  this->title = "RenderWidget";

  // The below block means that every time the updateTime expires, we do an
  // update on the widget. Later on, we call the start() method to start this
  // time at a fixed frequency.  Note that we do not start this timer until the
  // first time that showEvent() is called.
  this->updateTimer = new QTimer(this);
  QObject::connect(this->updateTimer, SIGNAL(timeout()), this, SLOT(update()));
  QObject::connect(this, SIGNAL(NewInitialModel(const ignition::msgs::Model &)),
    this, SLOT(SetInitialModel(const ignition::msgs::Model &)));
  QObject::connect(this,
    SIGNAL(NewDraw(const ignition::msgs::PosesStamped &)), this,
    SLOT(UpdateScene(const ignition::msgs::PosesStamped &)));

  this->node.Subscribe("/DRAKE_VIEWER_LOAD_ROBOT",
    &RenderWidget::OnInitialModel, this);
  this->node.Subscribe("/DRAKE_VIEWER_DRAW",
    &RenderWidget::OnUpdateScene, this);

  this->setMinimumHeight(100);
}

/////////////////////////////////////////////////
RenderWidget::~RenderWidget()
{
}

/////////////////////////////////////////////////
void RenderWidget::OnInitialModel(const ignition::msgs::Model &_msg)
{
  emit this->NewInitialModel(_msg);
}

/////////////////////////////////////////////////
void RenderWidget::OnUpdateScene(const ignition::msgs::PosesStamped &_msg)
{
  emit this->NewDraw(_msg);
}

/////////////////////////////////////////////////
bool RenderWidget::CreateVisual(const ignition::msgs::Visual &_vis,
  ignition::rendering::VisualPtr &_visual,
  ignition::rendering::MaterialPtr &_material) const
{
  _visual = this->scene->CreateVisual();
  if (!_visual) {
    ignerr << "Failed to create visual" << std::endl;
    return false;
  }

  _material = this->scene->CreateMaterial();
  if (!_material) {
    ignerr << "Failed to create material" << std::endl;
    return false;
  }

  if (_vis.has_material()) {
    const auto &material = _vis.material();
    if (material.has_diffuse()) {
      const auto &diffuse = material.diffuse();
      if (diffuse.has_r() && diffuse.has_g() && diffuse.has_b()) {
        _material->SetDiffuse(diffuse.r(), diffuse.g(), diffuse.b());
      }
      const auto &ambient = material.ambient();
      if (ambient.has_r() && ambient.has_g() && ambient.has_b()) {
        _material->SetAmbient(ambient.r(), ambient.g(), ambient.b());
      }
      const auto &specular = material.specular();
      if (specular.has_r() && specular.has_g() && specular.has_b()) {
        _material->SetSpecular(specular.r(), specular.g(), specular.b());
      }
    }
  }

  _material->SetShininess(50);
  _material->SetReflectivity(0);

  return true;
}

/////////////////////////////////////////////////
ignition::rendering::VisualPtr RenderWidget::Render(
  const ignition::msgs::Visual &_vis,
  const ignition::math::Vector3d &_scale,
  const ignition::rendering::MaterialPtr &_material,
  ignition::rendering::VisualPtr &_visual)
{
  setPoseFromMessage(_vis, _visual);
  _visual->SetLocalScale(_scale.X(), _scale.Y(), _scale.Z());
  _visual->SetMaterial(_material);
  this->scene->RootVisual()->AddChild(_visual);
  return _visual;
}

/////////////////////////////////////////////////
ignition::rendering::VisualPtr RenderWidget::RenderBox(
  const ignition::msgs::Visual &_vis,
  ignition::rendering::VisualPtr &_visual,
  ignition::rendering::MaterialPtr &_material)
{
  ignition::math::Vector3d scale = ignition::math::Vector3d::One;
  auto geomBox = _vis.geometry().box();
  if (geomBox.has_size()) {
    scale.X() = geomBox.size().z();
    scale.Y() = geomBox.size().x();
    scale.Z() = geomBox.size().y();
  }

  _visual->AddGeometry(scene->CreateBox());
  this->Render(_vis, scale, _material, _visual);
  return _visual;
}

/////////////////////////////////////////////////
ignition::rendering::VisualPtr RenderWidget::RenderSphere(
  const ignition::msgs::Visual &_vis,
  ignition::rendering::VisualPtr &_visual,
  ignition::rendering::MaterialPtr &_material)
{
  ignition::math::Vector3d scale = ignition::math::Vector3d::One;
  auto geomSphere = _vis.geometry().sphere();
  if (geomSphere.has_radius()) {
    scale.X() *= geomSphere.radius();
    scale.Y() *= geomSphere.radius();
    scale.Z() *= geomSphere.radius();
  }

  _visual->AddGeometry(scene->CreateSphere());
  this->Render(_vis, scale, _material, _visual);
  return _visual;
}

/////////////////////////////////////////////////
ignition::rendering::VisualPtr RenderWidget::RenderCylinder(
  const ignition::msgs::Visual &_vis,
  ignition::rendering::VisualPtr &_visual,
  ignition::rendering::MaterialPtr &_material)
{
  ignition::math::Vector3d scale = ignition::math::Vector3d::One;
  auto geomCylinder = _vis.geometry().cylinder();
  if (geomCylinder.has_radius()) {
    scale.X() *= geomCylinder.radius();
    scale.Y() *= geomCylinder.radius();
  }
  if (geomCylinder.has_length()) {
    scale.Z() = geomCylinder.length();
  }

  _visual->AddGeometry(scene->CreateCylinder());
  this->Render(_vis, scale, _material, _visual);
  return _visual;
}

/////////////////////////////////////////////////
ignition::rendering::VisualPtr RenderWidget::RenderMesh(
  const ignition::msgs::Visual &_vis)
{
  ignition::rendering::VisualPtr root = this->scene->RootVisual();

  // Create directional light
  ignition::rendering::DirectionalLightPtr light0 =
    this->scene->CreateDirectionalLight();
  light0->SetDirection(0.5, 0.5, -1);
  light0->SetDiffuseColor(0.8, 0.8, 0.8);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  ignition::rendering::VisualPtr mesh = this->scene->CreateVisual();
  if (!mesh) {
    ignerr << "Failed to create visual" << std::endl;
    return nullptr;
  }

  // ToDo: Add support for multiple paths.
  // ToDo: Figure out how to use bazel for generating paths from this project
  //   E.g.: the "media/" directory.
  std::string mediaPath;
  char *pathCStr = std::getenv("DELPHYNE_MEDIA_PATH");
  if (!pathCStr || *pathCStr == '\0')
  {
    ignerr << "DELPHYNE_MEDIA_PATH environmet variable not set. Meshes will not"
           << " work" << std::endl;
    return nullptr;
  }
  mediaPath = pathCStr;

  ignition::rendering::MeshDescriptor descriptor;
  descriptor.meshName = ignition::common::joinPaths(mediaPath, "duck.dae");
  ignition::common::MeshManager *meshManager =
    ignition::common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  ignition::rendering::MeshPtr meshGeom = this->scene->CreateMesh(descriptor);
  mesh->AddGeometry(meshGeom);

  setPoseFromMessage(_vis, mesh);

  root->AddChild(mesh);

  return mesh;
}

/////////////////////////////////////////////////
void RenderWidget::SetInitialModel(const ignition::msgs::Model &_msg)
{
  if (this->initializedScene) {
    return;
  }

  for (int i = 0; i < _msg.link_size(); ++i) {
    auto link = _msg.link(i);

    // Sanity check: Verify that the model contains the required Id.
    if (!link.has_id()) {
      ignerr << "No model Id on link [" << link.name() << "]. Skipping"
             << std::endl;
      continue;
    }

    // Sanity check: Verify that the link contains the required name.
    if (!link.has_name()) {
      ignerr << "No name on link, skipping" << std::endl;
      continue;
    }

    // Sanity check: Verify that the visual doesn't exist already.
    const auto &modelIt = this->allVisuals.find(link.id());
    if (modelIt != this->allVisuals.end()) {
      if (modelIt->second.find(link.name()) != modelIt->second.end()) {
        ignerr << "Duplicated link [" << link.name() << "] for model "
               << link.id() << ". Skipping" << std::endl;
        continue;
      }
    }

    if (link.visual_size() == 0) {
      ignerr << "No visuals for [" << link.name() << "]. Skipping" << std::endl;
      continue;
    }

    igndbg << "Rendering: [" << link.name() << "] (" << link.id() << ")"
           << std::endl;

    // Iterate through all the visuals of the link and store them.
    VisualPtr_V visuals;
    for (int j = 0; j < link.visual_size(); ++j) {

      const auto &vis = link.visual(j);
      if (!vis.has_geometry()) {
        ignerr << "No geometry in link [" << link.name() << "][" << j
               << "]. Skipping" << std::endl;
        continue;
      }

      ignition::rendering::VisualPtr visual;
      ignition::rendering::MaterialPtr material;
      if (!this->CreateVisual(vis, visual, material)) {
        continue;
      }

      ignition::rendering::VisualPtr ignvis;

      if (vis.geometry().has_box()) {
        ignvis = this->RenderBox(vis, visual, material);
      }
      else if (vis.geometry().has_sphere()) {
        ignvis = this->RenderSphere(vis, visual, material);
      }
      else if (vis.geometry().has_cylinder()) {
        ignvis = this->RenderCylinder(vis, visual, material);
      }
      else if (vis.geometry().has_mesh()) {
        ignvis = this->RenderMesh(vis);
      }
      else {
        ignerr << "Invalid shape for [" << link.name() << "]. Skipping"
               << std::endl;
        continue;
      }

      visuals.push_back(ignvis);
    }

    // Update the collection of visuals.
    auto &links = this->allVisuals[link.id()];
    links.insert(std::make_pair(link.name(), visuals));
  }

  this->initializedScene = true;
}

/////////////////////////////////////////////////
void RenderWidget::UpdateScene(const ignition::msgs::PosesStamped &_msg)
{
  for (int i = 0; i < _msg.pose_size(); ++i) {
    auto pose = _msg.pose(i);

    // Sanity check: It's required to have a model Id.
    if (!pose.has_id()) {
      ignerr << "Skipping pose " << pose.name() << " without id" << std::endl;
      continue;
    }

    // Sanity check: Make sure that the model Id exists.
    auto robotIt = this->allVisuals.find(pose.id());
    if (robotIt == this->allVisuals.end()) {
      ignerr << "Could not find model Id [" << pose.id() << "]. Skipping"
             << std::endl;
      continue;
    }

    // Sanity check: It's required to have a link name.
    if (!pose.has_name()) {
      ignerr << "Skipping pose without name" << std::endl;
      continue;
    }

    // Sanity check: Make sure that the link name exists.
    auto visualsIt = robotIt->second.find(pose.name());
    if (visualsIt == robotIt->second.end()) {
      ignerr << "Could not find link name [" << pose.name() << "]. Skipping"
             << std::endl;
      continue;
    }

    // Update all visuals of this link.
    auto &visuals = visualsIt->second;
    for (auto &visual : visuals) {
      // The setPoseFromMessage() assumes an ignition::msgs::Visual
      // message here, so we setup a dummy one to please it.
      ignition::msgs::Visual tmpvis;
      *tmpvis.mutable_pose() = pose;
      setPoseFromMessage(tmpvis, visual);
    }
  }
}

/////////////////////////////////////////////////
void RenderWidget::CreateRenderWindow()
{
  std::string engineName = "ogre";
  ignition::rendering::RenderEngineManager* manager =
    ignition::rendering::RenderEngineManager::Instance();
  ignition::rendering::RenderEngine* engine = manager->Engine(engineName);
  if (!engine) {
    ignerr << "Engine '" << engineName << "' is not supported" << std::endl;
    return;
  }

  // Create sample scene
  this->scene = engine->CreateScene("scene");
  if (!this->scene) {
    ignerr << "Failed to create scene" << std::endl;
    return;
  }

  this->scene->SetAmbientLight(0.3, 0.3, 0.3);
  ignition::rendering::VisualPtr root = this->scene->RootVisual();
  if (!root) {
    ignerr << "Failed to find the root visual" << std::endl;
    return;
  }
  ignition::rendering::DirectionalLightPtr light0 =
      this->scene->CreateDirectionalLight();
  if (!light0) {
    ignerr << "Failed to create a directional light" << std::endl;
    return;
  }
  light0->SetDirection(-0.5, -0.5, -1);
  light0->SetDiffuseColor(0.5, 0.5, 0.5);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  // create user camera
  this->camera = this->scene->CreateCamera("user_camera");
  if (!this->camera) {
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
  if (!this->renderWindow) {
    ignerr << "Failed to create camera render window" << std::endl;
    return;
  }
  this->renderWindow->SetHandle(winHandle);
  this->renderWindow->SetWidth(this->width());
  this->renderWindow->SetHeight(this->height());

  // render once to create the window.
  this->camera->Update();

  this->orbitViewControl.reset(new OrbitViewControl(this->camera));
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
QPaintEngine* RenderWidget::paintEngine() const {
  return nullptr;
}

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

/////////////////////////////////////////////////
void RenderWidget::mousePressEvent(QMouseEvent *_e) {
  if (!this->orbitViewControl) {
    return;
  }

  this->orbitViewControl->OnMousePress(_e);
}

/////////////////////////////////////////////////
void RenderWidget::mouseReleaseEvent(QMouseEvent *_e) {
  if (!this->orbitViewControl) {
    return;
  }

  this->orbitViewControl->OnMouseRelease(_e);
}

/////////////////////////////////////////////////
void RenderWidget::mouseMoveEvent(QMouseEvent *_e) {
  if (!this->orbitViewControl) {
    return;
  }

  this->orbitViewControl->OnMouseMove(_e);
}

/////////////////////////////////////////////////
void RenderWidget::wheelEvent(QWheelEvent *_e) {
  if (!this->orbitViewControl) {
    return;
  }

  this->orbitViewControl->OnMouseWheel(_e);
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(delphyne::gui::RenderWidget,
                                  ignition::gui::Plugin)
