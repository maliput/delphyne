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
#include <iterator>
#include <sstream>
#include <string>
#include <utility>
#include <tinyxml2.h>

#include <ignition/common/Console.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/gui/Iface.hh>
#include <ignition/gui/Plugin.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
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

Q_DECLARE_METATYPE(ignition::msgs::Model_V)

using namespace delphyne;
using namespace gui;

/////////////////////////////////////////////////
static void setPoseFromMessage(const ignition::msgs::Visual& _vis,
                               ignition::rendering::VisualPtr _shape) {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;

  if (_vis.has_pose()) {
    if (_vis.pose().has_position()) {
      x += _vis.pose().position().x();
      y += _vis.pose().position().y();
      z += _vis.pose().position().z();
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
std::string RenderWidget::FindFile(const std::string& _path) const {
  std::string res = "";
  // Case 1: Absolute path. E.g,: "/tmp/my_mesh.dae"
  if (ignition::common::StartsWith(_path, "/")) {
    if (ignition::common::exists(_path)) {
      res = _path;
    }
    return res;
  }

  int index = _path.find("://");
  std::string path = _path;

  // Case 2: Contains a prefix. E.g.: "package://meshes/my_mesh.dae"
  if (index != std::string::npos) {
    // Remove the prefix.
    path = _path.substr(index + 3, _path.size() - index - 3);
  }

  // Case 3: Relative path. E.g.: "meshes/my_mesh.dae"
  return ignition::common::SystemPaths::LocateLocalFile(path,
                                                        this->packagePaths);
}

/////////////////////////////////////////////////
RenderWidget::RenderWidget(QWidget* parent)
    : Plugin(), initializedScene(false), engine(nullptr) {
  qRegisterMetaType<ignition::msgs::Model_V>();

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
  QObject::connect(
      this, SIGNAL(NewInitialModel(const ignition::msgs::Model_V&)), this,
      SLOT(SetInitialModels(const ignition::msgs::Model_V&)));
  QObject::connect(this, SIGNAL(NewDraw(const ignition::msgs::Model_V&)), this,
                   SLOT(UpdateScene(const ignition::msgs::Model_V&)));

  auto paths =
      ignition::common::SystemPaths::PathsFromEnv("DELPHYNE_PACKAGE_PATH");
  if (paths.empty()) {
    ignerr << "DELPHYNE_PACKAGE_PATH environment variable is not set"
           << std::endl;
  }
  std::copy(paths.begin(), paths.end(), std::back_inserter(this->packagePaths));

  this->node.Subscribe("/DRAKE_VIEWER_LOAD_ROBOT",
                       &RenderWidget::OnInitialModel, this);
  this->node.Subscribe("/DRAKE_VIEWER_DRAW", &RenderWidget::OnUpdateScene,
                       this);

  this->setMinimumHeight(100);
}

/////////////////////////////////////////////////
RenderWidget::~RenderWidget() {
  if (this->engine != nullptr) {
    // TODO(clalancette): We need to call this->engine->Fini() to clean up
    // some engine resources, but this sometimes causes a hang on quit.
    // For right now, disable this, but we should debug this and re-enable this
    // cleanup.
    //this->engine->Fini();
  }
}

/////////////////////////////////////////////////
void RenderWidget::LoadConfig(const tinyxml2::XMLElement* _pluginElem) {
  tinyxml2::XMLPrinter printer;
  if (!_pluginElem->Accept(&printer)) {
    ignwarn << "There was an error parsing the plugin element for ["
            << this->title << "]." << std::endl;
    return;
  }

  // Load the user camera options.
  auto userCameraXML = _pluginElem->FirstChildElement("camera");
  while (userCameraXML) {
    std::string cameraName;
    if (userCameraXML->Attribute("name", "user_camera")) {
      auto poseXML = userCameraXML->FirstChildElement("pose");
      if (poseXML) {
        auto poseStr = poseXML->GetText();
        std::istringstream stream(poseStr);
        stream >> this->userSettings.userCameraPose;
      } else {
        ignerr << "Unable to parse <pose> element within <camera>" << std::endl;
      }
      break;
    }

    // Not a user camera, skip to the next camera.
    userCameraXML = userCameraXML->NextSiblingElement("camera");
  }
}

/////////////////////////////////////////////////
std::string RenderWidget::ConfigStr() const {
  tinyxml2::XMLDocument xmlDoc;

  // Create the plugin element.
  tinyxml2::XMLElement* pluginXML = xmlDoc.NewElement("plugin");
  pluginXML->SetAttribute("filename", "RenderWidget");
  xmlDoc.InsertFirstChild(pluginXML);

  // User camera options.
  tinyxml2::XMLElement* userCameraXML = xmlDoc.NewElement("camera");
  userCameraXML->SetAttribute("name", "user_camera");
  pluginXML->InsertEndChild(userCameraXML);
  tinyxml2::XMLElement* poseXML = xmlDoc.NewElement("pose");
  auto pos = this->camera->LocalPose().Pos();
  auto rot = this->camera->LocalPose().Rot().Euler();
  std::stringstream stream;
  stream << pos.X() << " " << pos.Y() << " " << pos.Z() << " " << rot.X() << " "
         << rot.Y() << " " << rot.Z();
  poseXML->SetText(stream.str().c_str());
  userCameraXML->InsertEndChild(poseXML);

  tinyxml2::XMLPrinter printer;
  xmlDoc.Print(&printer);

  return printer.CStr();
}

/////////////////////////////////////////////////
void RenderWidget::OnInitialModel(const ignition::msgs::Model_V& _msg) {
  emit this->NewInitialModel(_msg);
}

/////////////////////////////////////////////////
void RenderWidget::OnUpdateScene(const ignition::msgs::Model_V& _msg) {
  emit this->NewDraw(_msg);
}

/////////////////////////////////////////////////
bool RenderWidget::CreateVisual(
    const ignition::msgs::Visual& _vis, ignition::rendering::VisualPtr& _visual,
    ignition::rendering::MaterialPtr& _material) const {
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
    const auto& material = _vis.material();
    if (material.has_diffuse()) {
      const auto& diffuse = material.diffuse();
      if (diffuse.has_r() && diffuse.has_g() && diffuse.has_b()) {
        _material->SetDiffuse(diffuse.r(), diffuse.g(), diffuse.b());
      }
      const auto& ambient = material.ambient();
      if (ambient.has_r() && ambient.has_g() && ambient.has_b()) {
        _material->SetAmbient(ambient.r(), ambient.g(), ambient.b());
      }
      const auto& specular = material.specular();
      if (specular.has_r() && specular.has_g() && specular.has_b()) {
        _material->SetSpecular(specular.r(), specular.g(), specular.b());
      }
    }
  }

  if (_vis.has_transparency()) {
    _material->SetTransparency(_vis.transparency());
  }

  _material->SetShininess(50);
  _material->SetReflectivity(0);

  return true;
}

/////////////////////////////////////////////////
ignition::rendering::VisualPtr RenderWidget::Render(
    const ignition::msgs::Visual& _vis, const ignition::math::Vector3d& _scale,
    const ignition::rendering::MaterialPtr& _material,
    ignition::rendering::VisualPtr& _visual) {
  setPoseFromMessage(_vis, _visual);
  _visual->SetLocalScale(_scale.X(), _scale.Y(), _scale.Z());
  _visual->SetMaterial(_material);
  return _visual;
}

/////////////////////////////////////////////////
ignition::rendering::VisualPtr RenderWidget::RenderBox(
    const ignition::msgs::Visual& _vis, ignition::rendering::VisualPtr& _visual,
    ignition::rendering::MaterialPtr& _material) {
  ignition::math::Vector3d scale = ignition::math::Vector3d::One;
  auto geomBox = _vis.geometry().box();
  if (geomBox.has_size()) {
    scale.X() = geomBox.size().x();
    scale.Y() = geomBox.size().y();
    scale.Z() = geomBox.size().z();
  }

  _visual->AddGeometry(this->scene->CreateBox());
  this->Render(_vis, scale, _material, _visual);
  return _visual;
}

/////////////////////////////////////////////////
ignition::rendering::VisualPtr RenderWidget::RenderSphere(
    const ignition::msgs::Visual& _vis, ignition::rendering::VisualPtr& _visual,
    ignition::rendering::MaterialPtr& _material) {
  ignition::math::Vector3d scale = ignition::math::Vector3d::One;
  auto geomSphere = _vis.geometry().sphere();
  if (geomSphere.has_radius()) {
    scale.X() *= geomSphere.radius();
    scale.Y() *= geomSphere.radius();
    scale.Z() *= geomSphere.radius();
  }

  _visual->AddGeometry(this->scene->CreateSphere());
  this->Render(_vis, scale, _material, _visual);
  return _visual;
}

/////////////////////////////////////////////////
ignition::rendering::VisualPtr RenderWidget::RenderCylinder(
    const ignition::msgs::Visual& _vis, ignition::rendering::VisualPtr& _visual,
    ignition::rendering::MaterialPtr& _material) {
  ignition::math::Vector3d scale = ignition::math::Vector3d::One;
  auto geomCylinder = _vis.geometry().cylinder();
  if (geomCylinder.has_radius()) {
    scale.X() *= 2 * geomCylinder.radius();
    scale.Y() *= 2 * geomCylinder.radius();
  }
  if (geomCylinder.has_length()) {
    scale.Z() = geomCylinder.length();
  }

  _visual->AddGeometry(this->scene->CreateCylinder());
  this->Render(_vis, scale, _material, _visual);
  return _visual;
}

/////////////////////////////////////////////////
void RenderWidget::RenderGroundPlane() {
  auto material = this->scene->CreateMaterial();
  if (!material) {
    ignerr << "Failed to create ground plane material" << std::endl;
    return;
  }

  material->SetShininess(50);
  material->SetReflectivity(0);

  auto groundPlaneVisual = this->scene->CreateVisual();
  if (!groundPlaneVisual) {
    ignerr << "Failed to create ground plane visual" << std::endl;
    return;
  }

  // 100 x 100 ground plane.
  groundPlaneVisual->SetLocalScale(100, 100, 1);
  groundPlaneVisual->AddGeometry(scene->CreatePlane());
  groundPlaneVisual->SetMaterial(material);
  this->scene->RootVisual()->AddChild(groundPlaneVisual);
}

/////////////////////////////////////////////////
void RenderWidget::RenderGrid() {
  ignition::msgs::Visual gridVisual;
  auto* gridGeometry = gridVisual.mutable_geometry();
  gridGeometry->set_type(ignition::msgs::Geometry::MESH);

  auto* gridMesh = gridGeometry->mutable_mesh();
  gridMesh->set_filename("media/grid.obj");

  ignition::rendering::VisualPtr gridVisualPtr;
  gridVisualPtr = this->RenderMesh(gridVisual);
  this->scene->RootVisual()->AddChild(gridVisualPtr);
}

/////////////////////////////////////////////////
void RenderWidget::RenderOrigin() {
  const double kAxisRadius = 0.02;
  const double kAxisLength = 10000;
  const double kAxisHalfLength = kAxisLength / 2.0;

  // Create the visual axes.
  std::array<ignition::rendering::VisualPtr, 3> axes;
  for (auto& axis : axes) {
    axis = this->scene->CreateVisual();
    if (!axis) {
      ignerr << "Failed to create axis visual" << std::endl;
      return;
    }
    axis->SetLocalScale(kAxisRadius, kAxisRadius, kAxisLength);
    axis->AddGeometry(scene->CreateCylinder());
  }

  const ignition::math::Pose3d kAxisPoseX(kAxisHalfLength, 0, 0, 0, IGN_PI_2,
                                          0);
  axes[0]->SetLocalPose(kAxisPoseX);
  axes[0]->SetMaterial("Default/TransRed");
  const ignition::math::Pose3d kAxisPoseY(0, kAxisHalfLength, 0, IGN_PI_2, 0,
                                          0);
  axes[1]->SetLocalPose(kAxisPoseY);
  axes[1]->SetMaterial("Default/TransGreen");
  const ignition::math::Pose3d kAxisPoseZ(0, 0, kAxisHalfLength, 0, 0, 0);
  axes[2]->SetLocalPose(kAxisPoseZ);
  axes[2]->SetMaterial("Default/TransBlue");

  for (auto& axis : axes) {
    this->scene->RootVisual()->AddChild(axis);
  }
}

/////////////////////////////////////////////////
ignition::rendering::VisualPtr RenderWidget::RenderMesh(
    const ignition::msgs::Visual& _vis) {
  // Sanity check: Make sure that the message contains all required fields.
  if (!_vis.has_geometry()) {
    ignerr << "Unable to find geometry in message" << std::endl;
  }

  if (!_vis.geometry().has_mesh()) {
    ignerr << "Unable to find mesh in message" << std::endl;
  }

  if (!_vis.geometry().mesh().has_filename()) {
    ignerr << "Unable to find filename in message" << std::endl;
  }

  ignition::rendering::VisualPtr mesh = this->scene->CreateVisual();
  if (!mesh) {
    ignerr << "Failed to create visual" << std::endl;
    return nullptr;
  }

  auto filename = _vis.geometry().mesh().filename();
  ignition::rendering::MeshDescriptor descriptor;
  descriptor.meshName = this->FindFile(filename);
  if (descriptor.meshName.empty()) {
    ignerr << "Unable to locate mesh [" << filename << "]" << std::endl;
    return nullptr;
  }
  ignition::common::MeshManager* meshManager =
      ignition::common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  if (!descriptor.mesh) {
    return nullptr;
  }

  ignition::rendering::MeshPtr meshGeom = this->scene->CreateMesh(descriptor);
  mesh->AddGeometry(meshGeom);

  setPoseFromMessage(_vis, mesh);

  ignition::rendering::VisualPtr root = this->scene->RootVisual();

  return mesh;
}

/////////////////////////////////////////////////
void RenderWidget::SetInitialModels(const ignition::msgs::Model_V& _msg) {
  if (this->initializedScene) {
    return;
  }

  for (int i = 0; i < _msg.models_size(); ++i) {
    LoadModel(_msg.models(i));
  }
}

/////////////////////////////////////////////////
ignition::rendering::VisualPtr RenderWidget::CreateLinkRootVisual(
    ignition::msgs::Link& _link, uint32_t _robotID) {
  ignition::rendering::VisualPtr linkRootVisual = this->scene->CreateVisual();
  // The visual's root pose is initialized to zero because the load message
  // from LCM doesn't include the link pose but only the poses of the inner
  // visuals.
  // This value will be updated on each new draw message received.
  ignition::math::Pose3d newpose(0, 0, 0, 1, 0, 0, 0);
  linkRootVisual->SetLocalPose(newpose);
  this->scene->RootVisual()->AddChild(linkRootVisual);

  // Assign the visual created above as the root visual of the link
  auto& links = this->allVisuals[_robotID];
  links.insert(std::make_pair(_link.name(), linkRootVisual));
  return linkRootVisual;
}

/////////////////////////////////////////////////
void RenderWidget::LoadModel(const ignition::msgs::Model& _msg) {
  // Sanity check: It's required to have a model Id.
  if (!_msg.has_id()) {
    ignerr << "Skipping model without id" << std::endl;
    return;
  }

  for (int i = 0; i < _msg.link_size(); ++i) {
    auto link = _msg.link(i);

    // Sanity check: Verify that the link contains the required name.
    if (!link.has_name()) {
      ignerr << "No name on link, skipping" << std::endl;
      continue;
    }

    // Sanity check: Verify that the visual doesn't exist already.
    const auto& modelIt = this->allVisuals.find(_msg.id());
    if (modelIt != this->allVisuals.end()) {
      if (modelIt->second.find(link.name()) != modelIt->second.end()) {
        ignerr << "Duplicated link [" << link.name() << "] for model "
               << _msg.id() << ". Skipping" << std::endl;
        continue;
      }
    }

    if (link.visual_size() == 0) {
      ignerr << "No visuals for [" << link.name() << "]. Skipping" << std::endl;
      continue;
    }

    igndbg << "Rendering: [" << link.name() << "] (" << _msg.id() << ")"
           << std::endl;

    // Create a single root visual per link which will serve only
    // for hierarchical purposes, so that it can become the parent
    // of each of the "real" visuals of the link.
    ignition::rendering::VisualPtr linkRootVisual =
        RenderWidget::CreateLinkRootVisual(link, _msg.id());

    // Iterate through all the visuals of the link and add
    // them as childs of the link's root visual
    for (int j = 0; j < link.visual_size(); ++j) {
      const auto& vis = link.visual(j);
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

      // A real visual that is going to become
      // one of the link's root-visual childs
      ignition::rendering::VisualPtr ignvis;

      if (vis.geometry().has_box()) {
        ignvis = this->RenderBox(vis, visual, material);
      } else if (vis.geometry().has_sphere()) {
        ignvis = this->RenderSphere(vis, visual, material);
      } else if (vis.geometry().has_cylinder()) {
        ignvis = this->RenderCylinder(vis, visual, material);
      } else if (vis.geometry().has_mesh()) {
        ignvis = this->RenderMesh(vis);
      } else {
        ignerr << "Invalid shape for [" << link.name() << "]. Skipping"
               << std::endl;
        continue;
      }

      linkRootVisual->AddChild(ignvis);
    }
  }

  this->initializedScene = true;
}

/////////////////////////////////////////////////
void RenderWidget::UpdateScene(const ignition::msgs::Model_V& _msg) {
  for (int j = 0; j < _msg.models_size(); ++j) {
    auto model = _msg.models(j);

    // Sanity check: It's required to have a model Id.
    if (!model.has_id()) {
      ignerr << "Skipping model without id" << std::endl;
      continue;
    }

    for (int i = 0; i < model.link_size(); ++i) {
      auto link = model.link(i);

      // Sanity check: It's required to have a link name.
      if (!link.has_name()) {
        ignerr << "Skipping link without name" << std::endl;
        continue;
      }

      // Sanity check: Make sure that the model Id exists.
      auto robotIt = this->allVisuals.find(model.id());
      if (robotIt == this->allVisuals.end()) {
        ignerr << "Could not find model Id [" << model.id() << "]. Skipping"
               << std::endl;
        continue;
      }

      // Sanity check: Make sure that the link name exists.
      auto visualsIt = robotIt->second.find(link.name());
      if (visualsIt == robotIt->second.end()) {
        ignerr << "Could not find link name [" << link.name() << "]. Skipping"
               << std::endl;
        continue;
      }

      auto pose = link.pose();

      // Update the pose of the root visual only;
      // the relative poses of the children remain the same
      auto& visual = visualsIt->second;
      // The setPoseFromMessage() assumes an ignition::msgs::Visual
      // message here, so we setup a dummy one to please it.
      ignition::msgs::Visual tmpvis;
      *tmpvis.mutable_pose() = pose;
      setPoseFromMessage(tmpvis, visual);
    }
  }
}

/////////////////////////////////////////////////
void RenderWidget::CreateRenderWindow() {
  std::string engineName = "ogre";
  ignition::rendering::RenderEngineManager* manager =
      ignition::rendering::RenderEngineManager::Instance();
  this->engine = manager->Engine(engineName);
  if (!this->engine) {
    ignerr << "Engine '" << engineName << "' is not supported" << std::endl;
    return;
  }

  // Create sample scene
  this->scene = engine->CreateScene("scene");
  if (!this->scene) {
    ignerr << "Failed to create scene" << std::endl;
    return;
  }

  // Lights.
  this->scene->SetAmbientLight(0.9, 0.9, 0.9);
  ignition::rendering::VisualPtr root = this->scene->RootVisual();
  if (!root) {
    ignerr << "Failed to find the root visual" << std::endl;
    return;
  }
  auto directionalLight = this->scene->CreateDirectionalLight();
  if (!directionalLight) {
    ignerr << "Failed to create a directional light" << std::endl;
    return;
  }
  directionalLight->SetDirection(-0.5, -0.5, -1);
  directionalLight->SetDiffuseColor(0.9, 0.9, 0.9);
  directionalLight->SetSpecularColor(0.9, 0.9, 0.9);
  root->AddChild(directionalLight);

  // create user camera
  this->camera = this->scene->CreateCamera("user_camera");
  if (!this->camera) {
    ignerr << "Failed to create camera" << std::endl;
    return;
  }
  // Rotate 135 deg on the Z axis
  auto rotation = this->userSettings.userCameraPose.Rot().Euler();
  this->camera->SetLocalRotation(rotation.X(), rotation.Y(), rotation.Z());
  // Step away from the center of the scene
  auto position = this->userSettings.userCameraPose.Pos();
  this->camera->SetLocalPosition(position.X(), position.Y(), position.Z());
  this->camera->SetAspectRatio(static_cast<double>(this->width()) /
                               this->height());
  this->camera->SetHFOV(IGN_DTOR(60));
  root->AddChild(this->camera);

  this->scene->SetBackgroundColor(0.9, 0.9, 0.9);

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

  // Render the grid and the origin reference frame.
  this->RenderGrid();
  this->RenderOrigin();

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
  this->camera->SetAspectRatio(static_cast<double>(this->width()) /
                               this->height());
  // This is a bit janky. We need to update ign-rendering so that the
  // vertical FOV is auto updated when the aspect ratio is changed
  this->camera->SetHFOV(IGN_DTOR(60));
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
void RenderWidget::mousePressEvent(QMouseEvent* _e) {
  if (!this->orbitViewControl) {
    return;
  }

  this->orbitViewControl->OnMousePress(_e);
}

/////////////////////////////////////////////////
void RenderWidget::mouseReleaseEvent(QMouseEvent* _e) {
  if (!this->orbitViewControl) {
    return;
  }

  this->orbitViewControl->OnMouseRelease(_e);
}

/////////////////////////////////////////////////
void RenderWidget::mouseMoveEvent(QMouseEvent* _e) {
  if (!this->orbitViewControl) {
    return;
  }

  this->orbitViewControl->OnMouseMove(_e);
}

/////////////////////////////////////////////////
void RenderWidget::wheelEvent(QWheelEvent* _e) {
  if (!this->orbitViewControl) {
    return;
  }

  this->orbitViewControl->OnMouseWheel(_e);
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(delphyne::gui::RenderWidget,
                                  ignition::gui::Plugin)
