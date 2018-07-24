// Copyright 2018 Toyota Research Institute

#include "backend/data_logger.h"

#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include <ignition/common/Filesystem.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/log/Log.hh>

#include "delphyne/utility/package.h"

#include "helpers.h"

namespace delphyne {
namespace {

class DataLoggerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tmpdir_ = test::MakeTemporaryDirectory("/tmp/XXXXXX");
    auto package = std::make_unique<utility::BundledPackage>(
        ignition::common::joinPaths(tmpdir_, "base_package"));
    const std::string path_to_mesh =
        ignition::common::joinPaths(tmpdir_, "mesh.obj");
    std::fstream fs(path_to_mesh, std::ios::out);
    fs.close();
    package->Add(kSceneMeshURI, path_to_mesh);
    utility::PackageManager* package_manager =
        utility::PackageManager::Instance();
    package_manager->Use(std::move(package));
    logger_ = std::make_unique<DataLogger>();
  }

  // Builds up an ignition::msgs::Scene instance with a single
  // model that has a single link with a single mesh visual
  // geometry.
  ignition::msgs::Scene BuildDummySceneMessage() {
    ignition::msgs::Scene scene_msg;
    ignition::msgs::Model* model_msg = scene_msg.add_model();
    ignition::msgs::Link* link_msg = model_msg->add_link();
    ignition::msgs::Visual* visual_msg = link_msg->add_visual();
    ignition::msgs::Geometry* geometry_msg = visual_msg->mutable_geometry();
    ignition::msgs::MeshGeom* mesh_msg = geometry_msg->mutable_mesh();
    mesh_msg->set_filename(kSceneMeshURI);
    return scene_msg;
  }

  void TearDown() override {
    logger_.reset();
    ignition::common::removeAll(tmpdir_);
  }

  std::string tmpdir_{""};
  std::unique_ptr<DataLogger> logger_{nullptr};
  const std::string kSceneMeshURI{"package://dummy/mesh"};
};

// Checks that failing to meet DataLogger methods'
// preconditions results in an error.
TEST_F(DataLoggerTest, LoggingPreconditions) {
  // Checks that trying to capture meshes on a logger
  // that has not been started fails.
  EXPECT_THROW(logger_->CaptureMeshes(
      ignition::msgs::Scene()),
      std::runtime_error);
  // Checks that trying to stop a logger that has not
  // been started fails.
  EXPECT_THROW(logger_->Stop(), std::runtime_error);

  const std::string path_to_logfile =
      ignition::common::joinPaths(tmpdir_, "test.log");
  EXPECT_TRUE(logger_->Start(path_to_logfile));
  
  // Checks that trying to start a logger that has
  // already been started fails.
  EXPECT_THROW(logger_->Start(path_to_logfile), std::runtime_error);
}

// Checks that logging is working as expected.
TEST_F(DataLoggerTest, LoggingWorkflow) {
  const std::string path_to_logfile =
      ignition::common::joinPaths(tmpdir_, "test.log");
  EXPECT_FALSE(ignition::common::exists(path_to_logfile));

  EXPECT_TRUE(logger_->Start(path_to_logfile));
  EXPECT_EQ(logger_->logpath(), path_to_logfile);

  EXPECT_TRUE(logger_->CaptureMeshes(BuildDummySceneMessage()));

  logger_->Stop();

  EXPECT_TRUE(ignition::common::isFile(path_to_logfile));

  const std::string path_to_logdir =
      ignition::common::joinPaths(tmpdir_, "test");
  EXPECT_TRUE(ignition::common::createDirectories(path_to_logdir));
  EXPECT_EQ(detail::unzipDirectory(path_to_logfile, path_to_logdir), 0);

  const std::string topic_log_path =
      ignition::common::joinPaths(path_to_logdir, "topic.db");
  EXPECT_TRUE(ignition::common::isFile(topic_log_path));
  ignition::transport::log::Log topic_log;
  EXPECT_TRUE(topic_log.Open(topic_log_path));

  const std::string bundled_package_path =
      ignition::common::joinPaths(path_to_logdir, "bundle");
  EXPECT_TRUE(ignition::common::isDirectory(bundled_package_path));
  utility::BundledPackage package(bundled_package_path);
  EXPECT_TRUE(package.Exists(kSceneMeshURI));
}

}  // namespace
}  // namespace delphyne
