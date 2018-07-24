// Copyright 2017 Toyota Research Institute

#include "delphyne/utility/package.h"

#include <fstream>
#include <string>
#include <sstream>

#include <gtest/gtest.h>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/URI.hh>

#include "helpers.h"

namespace delphyne {
namespace utility {
namespace {

TEST(ToURITests, HandlesAbsolutePaths) {
  const ignition::common::URI uri = ToURI("/absolute/path");
  EXPECT_EQ(uri.Str(), "file://absolute/path");
}

TEST(ToURITests, HandlesRelativePaths) {
  const ignition::common::URI uri = ToURI("relative/path");
  EXPECT_EQ(uri.Str(), "package://relative/path");
}

TEST(ToURITests, HandlesURIs) {
  const ignition::common::URI uri = ToURI("git://repository");
  EXPECT_EQ(uri.Str(), "git://repository");
}

class PackageTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Make dummy package for testing.
    using ignition::common::joinPaths;
    tmpdir_ = test::MakeTemporaryDirectory("/tmp/XXXXXX");
    path_to_package_ = joinPaths(tmpdir_, "package_root");
    path_to_package_resources_ = joinPaths(path_to_package_, "package");
    ignition::common::createDirectories(path_to_package_resources_);
    path_to_resource_ = joinPaths(path_to_package_resources_, "stuff");
    std::fstream resource(path_to_resource_, std::ios::out);
    resource.close();
    // Setup environment to point to the built dummy package.
    setenv("DELPHYNE_PACKAGE_PATH", path_to_package_resources_.c_str(), 1);
  }

  void TestPackageUseCases(const Package& package) {
    EXPECT_FALSE(package.Exists(kNonExistentResourceURI));
    EXPECT_TRUE(package.Exists(kResourceURI));
    EXPECT_EQ(package.Find(kResourceURI), path_to_resource_);
  }

  void TearDown() override {
    ignition::common::removeAll(tmpdir_);
  }

  std::string tmpdir_{};
  std::string path_to_package_{};
  std::string path_to_package_resources_{};
  std::string path_to_resource_{};

  const std::string kResourceURI{"package://stuff"};
  const std::string kNonExistentResourceURI{"package://missing"};
};

// Checks that querying a system package for resources works as expected.
TEST_F(PackageTest, UsingSystemEnvironment) {
  SystemPackage package;
  TestPackageUseCases(package);
}

// Checks that querying a bundled package for resources works as expected.
TEST_F(PackageTest, UsingBundledPackage) {
  BundledPackage package(path_to_package_);
  TestPackageUseCases(package);
}

// Checks that adding resources to a bundled package works as expected.
TEST_F(PackageTest, AddingToBundledPackage) {
  const std::string path_to_new_resource =
      ignition::common::joinPaths(tmpdir_, "new_stuff");
  std::fstream new_resource(path_to_new_resource, std::ios::out);
  new_resource << "dummy";
  new_resource.flush();
  new_resource.close();

  BundledPackage package(path_to_package_);

  const std::string kNewResourceURI = "file:///new/stuff";
  EXPECT_TRUE(package.Add(kNewResourceURI, path_to_new_resource));
  EXPECT_TRUE(package.Exists(kNewResourceURI));

  std::fstream packaged_resource(package.Find(kNewResourceURI));
  std::string content = "";
  packaged_resource >> content;
  EXPECT_EQ(content, "dummy");
}

}  // namespace
}  // namespace utility
}  // namespace delphyne
