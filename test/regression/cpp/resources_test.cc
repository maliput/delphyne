// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/URI.hh>

#include "delphyne/utility/package.h"
#include "test_utilities/helpers.h"

namespace delphyne {
namespace utility {
namespace {

class ResourceInspectionTest : public test::TestWithFiles {
 protected:
  void DoSetUp() override {
    // Setup environment to point to the temporary directory.
    setenv("DELPHYNE_PACKAGE_PATH", tmpdir().c_str(), 1);
  }

  void DoTearDown() override { unsetenv("DELPHYNE_PACKAGE_PATH"); }
};

TEST_F(ResourceInspectionTest, ColladaMeshInspection) {
  const std::string path_to_mesh_file = ignition::common::joinPaths(tmpdir(), "quad.dae");
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

  const ignition::common::URI mesh_uri("file://" + path_to_mesh_file);
  const ResourceInspector* resource_inspector = ResourceInspector::Instance();
  ASSERT_TRUE(resource_inspector != nullptr);
  const std::vector<ignition::common::URI> dep_uris = resource_inspector->GetDependencies(mesh_uri);
  ASSERT_EQ(static_cast<int>(dep_uris.size()), 1);
  EXPECT_EQ(dep_uris[0].Scheme(), "file");
  const std::string path_to_dep = dep_uris[0].Path().Str();
  const std::string path_to_texture = ignition::common::joinPaths(tmpdir(), "quad.png");
  EXPECT_EQ(path_to_texture, path_to_dep);
}

TEST_F(ResourceInspectionTest, OBJMeshInspection) {
  const std::string path_to_mesh_file = ignition::common::joinPaths(tmpdir(), "quad.obj");
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

  const ignition::common::URI mesh_uri("file://" + path_to_mesh_file);
  const ResourceInspector* resource_inspector = ResourceInspector::Instance();
  ASSERT_TRUE(resource_inspector != nullptr);
  const std::vector<ignition::common::URI> dep_uris = resource_inspector->GetDependencies(mesh_uri);
  ASSERT_EQ(static_cast<int>(dep_uris.size()), 1);
  EXPECT_EQ(dep_uris[0].Scheme(), "file");
  const std::string path_to_dep = dep_uris[0].Path().Str();
  const std::string path_to_mtllib = ignition::common::joinPaths(tmpdir(), "quad.mtl");
  EXPECT_EQ(path_to_mtllib, path_to_dep);
}

TEST_F(ResourceInspectionTest, MTLMaterialInspection) {
  const std::string path_to_material_file = ignition::common::joinPaths(tmpdir(), "demo.mtl");
  std::ofstream material_fs(path_to_material_file);
  material_fs << "newmtl demo"
              << "Ka 0.588 0.588 0.588"
              << "Kd 1 1 1"
              << "Ks 0.9 0.9 0.9"
              << "illum 2"
              << "Ns 14.9285"
              << "map_Kd green_grass.jpg"
              << "map_bump"
              << "bump"
              << "map_opacity"
              << "map_d"
              << "refl"
              << "map_kS"
              << "map_kA"
              << "map_Ns";
  material_fs.flush();
  material_fs.close();

  const ignition::common::URI material_uri("file://" + path_to_material_file);
  const ResourceInspector* resource_inspector = ResourceInspector::Instance();
  ASSERT_TRUE(resource_inspector != nullptr);
  const std::vector<ignition::common::URI> dep_uris = resource_inspector->GetDependencies(material_uri);
  ASSERT_EQ(static_cast<int>(dep_uris.size()), 1);
  EXPECT_EQ(dep_uris[0].Scheme(), "file");
  const std::string path_to_dep = dep_uris[0].Path().Str();
  const std::string path_to_map = ignition::common::joinPaths(tmpdir(), "green_grass.jpg");
  EXPECT_EQ(path_to_map, path_to_dep);
}

}  // namespace
}  // namespace utility
}  // namespace delphyne
