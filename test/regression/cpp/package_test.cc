// Copyright 2018 Toyota Research Institute

#include "delphyne/utility/package.h"

#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/URI.hh>

#include "test_utilities/helpers.h"

namespace delphyne {
namespace utility {
namespace {

TEST(ToURITests, HandlesAbsolutePaths) {
  const ignition::common::URI uri = ToURI("/absolute/path");
  EXPECT_EQ(uri.Str(), "file:///absolute/path");
}

TEST(ToURITests, HandlesRelativePaths) {
  const ignition::common::URI uri = ToURI("relative/path");
  EXPECT_EQ(uri.Str(), "package://relative/path");
}

TEST(ToURITests, HandlesURIs) {
  const ignition::common::URI uri = ToURI("git://repository");
  EXPECT_EQ(uri.Str(), "git://repository");
}

class PackageTest : public test::TestWithFiles {
 protected:
  void DoSetUp() override {
    // Make dummy package for testing.
    using ignition::common::joinPaths;
    path_to_package_ = joinPaths(tmpdir(), "this_package");
    path_to_package_resources_ = joinPaths(path_to_package_, "package");
    ignition::common::createDirectories(path_to_package_resources_);
    path_to_resource_ = joinPaths(path_to_package_resources_, "stuff");
    std::fstream resource(path_to_resource_, std::ios::out);
    // Setup environment to point to the built dummy package. It'll also
    // be used to setup the default package if the PackageManager is used
    // by any of the implementations.
    setenv("DELPHYNE_PACKAGE_PATH", path_to_package_resources_.c_str(), 1);
  }

  void TestPackageUseCases(const Package& package) {
    EXPECT_FALSE(package.Resolve(kNonExistentResourceURI).Valid());
    const ignition::common::URI resolved_uri = package.Resolve(kResourceURI);
    EXPECT_TRUE(resolved_uri.Valid());
    EXPECT_EQ(resolved_uri.Scheme(), "file");
    const std::string resolved_path = resolved_uri.Path().Str();
    EXPECT_EQ(resolved_path, path_to_resource_);
  }

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
  using ignition::common::joinPaths;
  const std::string path_to_new_resource = joinPaths(tmpdir(), "new_stuff");
  std::fstream new_resource(path_to_new_resource, std::ios::out);
  new_resource << "dummy";
  new_resource.flush();
  new_resource.close();

  BundledPackage package(path_to_package_);

  package.Add(path_to_new_resource);

  const ignition::common::URI resolved_uri = package.Resolve(path_to_new_resource);
  EXPECT_TRUE(resolved_uri.Valid());
  EXPECT_EQ(resolved_uri.Scheme(), "file");

  const std::string resolved_path = resolved_uri.Path().Str();
  std::fstream packaged_resource(resolved_path);
  std::string content = "";
  packaged_resource >> content;
  EXPECT_EQ(content, "dummy");
}

}  // namespace
}  // namespace utility
}  // namespace delphyne
