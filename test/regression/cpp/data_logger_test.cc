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

#include "utility/compression.h"
#include "delphyne/utility/package.h"
#include "test_utilities/helpers.h"

namespace delphyne {
namespace {

class DataLoggerTest : public test::TestWithFiles {
 protected:
  void DoSetUp() override {
    auto package = std::make_unique<utility::BundledPackage>(
        ignition::common::joinPaths(tmpdir(), "base"));
    path_to_mesh_mtl_ = ignition::common::joinPaths(tmpdir(), "quad.mtl");
    std::fstream matfs(path_to_mesh_mtl_, std::ios::out);
    matfs << "newmtl material0\n"
          << "Ka 1.000000 1.000000 1.000000\n"
          << "Kd 1.000000 1.000000 1.000000\n"
          << "Ks 0.000000 0.000000 0.000000\n"
          << "Tr 1.000000\n"
          << "illum 1\n"
          << "Ns 0.000000";
    matfs.close();
    path_to_mesh_ = ignition::common::joinPaths(tmpdir(), "quad.obj");
    std::ofstream meshfs(path_to_mesh_);
    meshfs << "# quad.obj\n"
           << "v 0.000000 0.000000 0.000000\n"
           << "v 0.000000 1.000000 0.000000\n"
           << "v 1.000000 1.000000 0.000000\n"
           << "v 1.000000 0.000000 0.000000\n"
           << "\n"
           << "vn 0.000000 0.000000 -1.000000\n"
           << "vn 0.000000 0.000000 -1.000000\n"
           << "vn 0.000000 0.000000 -1.000000\n"
           << "vn 0.000000 0.000000 -1.000000\n"
           << "\n"
           << "vt 0.000000 0.000000 0.000000\n"
           << "vt 0.000000 1.000000 0.000000\n"
           << "vt 1.000000 1.000000 0.000000\n"
           << "vt 1.000000 0.000000 0.000000\n"
           << "\n"
           << "mtllib quad.mtl\n"
           << "usemtl material0\n"
           << "f 1/1/1 2/2/2 3/3/3\n"
           << "f 1/1/1 3/3/3 4/4/4";
    meshfs.close();
    logger_ = std::make_unique<DataLogger>();
    // Setup environment to point to the temporary directory.
    setenv("DELPHYNE_PACKAGE_PATH", tmpdir().c_str(), 1);
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
    mesh_msg->set_filename(path_to_mesh_);
    return scene_msg;
  }

  void DoTearDown() override { logger_.reset(); }

  std::string path_to_mesh_{""};
  std::string path_to_mesh_mtl_{""};
  std::unique_ptr<DataLogger> logger_{nullptr};
};

// Checks that failing to meet DataLogger methods'
// preconditions results in an error.
TEST_F(DataLoggerTest, LoggingPreconditions) {
  // Checks that trying to capture meshes on a logger
  // that has not been started fails.
  EXPECT_THROW(logger_->CaptureMeshes(ignition::msgs::Scene()),
               std::runtime_error);
  // Checks that trying to stop a logger that has not
  // been started fails.
  EXPECT_THROW(logger_->Stop(), std::runtime_error);

  const std::string path_to_logfile =
      ignition::common::joinPaths(tmpdir(), "test.log");
  logger_->Start(path_to_logfile);

  // Checks that trying to start a logger that has
  // already been started fails.
  EXPECT_THROW(logger_->Start(path_to_logfile), std::runtime_error);
}

// Checks that logging is working as expected.
TEST_F(DataLoggerTest, LoggingWorkflow) {
  const std::string path_to_logfile =
      ignition::common::joinPaths(tmpdir(), "test.log");
  EXPECT_FALSE(ignition::common::exists(path_to_logfile));

  logger_->Start(path_to_logfile);
  EXPECT_EQ(logger_->logpath(), path_to_logfile);

  logger_->CaptureMeshes(BuildDummySceneMessage());

  logger_->Stop();

  EXPECT_TRUE(ignition::common::isFile(path_to_logfile));

  const std::string path_to_logdir =
      ignition::common::joinPaths(tmpdir(), "test");
  EXPECT_TRUE(ignition::common::createDirectories(path_to_logdir));
  UnzipDirectory(path_to_logfile, path_to_logdir);

  const std::string topic_log_path =
      ignition::common::joinPaths(path_to_logdir, "topic.db");
  EXPECT_TRUE(ignition::common::isFile(topic_log_path));
  ignition::transport::log::Log topic_log;
  EXPECT_TRUE(topic_log.Open(topic_log_path));

  const std::string bundled_package_path =
      ignition::common::joinPaths(path_to_logdir, "bundle");
  EXPECT_TRUE(ignition::common::isDirectory(bundled_package_path));
  utility::BundledPackage package(bundled_package_path);
  EXPECT_TRUE(package.Resolve(path_to_mesh_).Valid());
  EXPECT_TRUE(package.Resolve(path_to_mesh_mtl_).Valid());
}

}  // namespace
}  // namespace delphyne
