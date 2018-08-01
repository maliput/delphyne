// Copyright 2017 Toyota Research Institute

#include "delphyne/utility/package.h"

#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/URI.hh>

#include "helpers.h"

namespace delphyne {
namespace utility {
namespace {

class ResourceInspectionTest : public test::TestWithFiles {
 protected:
  void DoSetUp() override {
    // Setup environment to point to the temporary directory.
    setenv("DELPHYNE_PACKAGE_PATH", tmpdir().c_str(), 1);
  }
};

TEST_F(ResourceInspectionTest, ColladaMeshInspection) {
  const std::string path_to_mesh_file =
      ignition::common::joinPaths(tmpdir(), "quad.dae");
  std::ofstream mesh_fs(path_to_mesh_file);
  mesh_fs << "<asset>\n"
          << "   <up_axis>Y_UP</up_axis>\n"
          << "</asset>\n"
          << "<library_images>\n"
          << "   <image id=\"texture0\" name=\"texture0\">\n"
          << "       <init_from>quad.png</init_from>\n"
          << "   </image>\n"
          << "</library_images>\n";
  mesh_fs.flush();
  mesh_fs.close();

  const ignition::common::URI mesh_uri(
      "file://" + path_to_mesh_file);
  const ResourceInspector* resource_inspector =
      ResourceInspector::Instance();
  ASSERT_TRUE(resource_inspector != nullptr);
  const std::vector<ignition::common::URI> dep_uris =
      resource_inspector->Depends(mesh_uri);
  ASSERT_EQ(dep_uris.size(), 1);
  EXPECT_EQ(dep_uris[0].Scheme(), "file");
  const std::string path_to_dep =
      "/" + dep_uris[0].Path().Str();
  const std::string path_to_texture =
      ignition::common::joinPaths(tmpdir(), "quad.png");
  EXPECT_EQ(path_to_texture, path_to_dep);
}

TEST_F(ResourceInspectionTest, OBJMeshInspection) {
  const std::string path_to_mesh_file =
      ignition::common::joinPaths(tmpdir(), "quad.obj");
  std::ofstream mesh_fs(path_to_mesh_file);
  mesh_fs << "v 0.000000 0.000000 0.000000\n"
         << "v 0.000000 1.000000 0.000000\n"
         << "v 1.000000 1.000000 0.000000\n"
         << "v 1.000000 0.000000 0.000000\n"
         << "\n"
         << "mtllib quad.mtl\n"
         << "usemtl material0\n"
         << "f 1/1/1 2/2/2 3/3/3\n"
         << "f 1/1/1 3/3/3 4/4/4";
  mesh_fs.flush();
  mesh_fs.close();

  const ignition::common::URI mesh_uri(
      "file://" + path_to_mesh_file);
  const ResourceInspector* resource_inspector =
      ResourceInspector::Instance();
  ASSERT_TRUE(resource_inspector != nullptr);
  const std::vector<ignition::common::URI> dep_uris =
      resource_inspector->Depends(mesh_uri);
  ASSERT_EQ(dep_uris.size(), 1);
  EXPECT_EQ(dep_uris[0].Scheme(), "file");
  const std::string path_to_dep =
      "/" + dep_uris[0].Path().Str();
  const std::string path_to_mtllib =
      ignition::common::joinPaths(tmpdir(), "quad.mtl");
  EXPECT_EQ(path_to_mtllib, path_to_dep);
}

}  // namespace
}  // namespace utility
}  // namespace delphyne
