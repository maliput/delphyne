// Copyright 2018 Toyota Research Institute

#include "common/filesystem.h"

#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <ignition/common/Filesystem.hh>

#include "helpers.h"

namespace delphyne {
namespace {

TEST(FileSystemTest, CheckForAbsolutePaths) {
  EXPECT_FALSE(IsAbsolutePath("test.txt"));
  EXPECT_TRUE(IsAbsolutePath("/tmp/test.txt"));
}

TEST(FileSystemTest, CheckForValidFilenames) {
  EXPECT_FALSE(IsValidFilepath("/tmp/test/"));
  EXPECT_TRUE(IsValidFilepath("/tmp/test"));
  EXPECT_FALSE(IsValidFilepath("/tmp/test/."));
  EXPECT_FALSE(IsValidFilepath("/tmp/.."));
}

TEST(FileSystemTest, ExtractDirectoryname) {
  EXPECT_EQ(Dirname("test.txt"), "");
  EXPECT_EQ(Dirname("/tmp/test.txt"), "/tmp/");
  EXPECT_EQ(Dirname("/test.txt"), "/");
  EXPECT_EQ(Dirname("/tmp/../test.txt"), "/tmp/../");
}

TEST(FileSystemTest, ExtractExtension) {
  using pair_of_strings = std::pair<std::string, std::string>;
  EXPECT_EQ(SplitExtension("test"), pair_of_strings("test", ""));
  EXPECT_EQ(SplitExtension("test.txt"), pair_of_strings("test", "txt"));
  EXPECT_EQ(SplitExtension("test.1.txt"), pair_of_strings("test.1", "txt"));
  EXPECT_EQ(SplitExtension("../test.txt"), pair_of_strings("../test", "txt"));
  EXPECT_EQ(SplitExtension("../test"), pair_of_strings("../test", ""));
  EXPECT_EQ(SplitExtension("/tmp/."), pair_of_strings("/tmp/.", ""));
}

TEST(FileSystemTest, WalkDirectory) {
  constexpr const bool kRecursive = true;
  constexpr const bool kNotRecursive = false;

  const std::string testdir = test::MakeTemporaryDirectory("/tmp/XXXXXX");
  const std::set<std::string> first_expected_paths{
    ignition::common::joinPaths(testdir, "etc"),
    ignition::common::joinPaths(testdir, "var")};
  const std::set<std::string> all_expected_paths{
    ignition::common::joinPaths(testdir, "etc"),
    ignition::common::joinPaths(testdir, "etc/init"),
    ignition::common::joinPaths(testdir, "var"),
    ignition::common::joinPaths(testdir, "var/log"),
    ignition::common::joinPaths(testdir, "var/log/stuff")};
  for (const std::string& path : all_expected_paths) {
    ignition::common::createDirectories(path);
  }

  std::set<std::string> walked_paths{};
  DirectoryWalkFn walkfn = [&walked_paths] (const std::string& path) {
    walked_paths.insert(path);
  };

  EXPECT_THROW(WalkDirectory("not-a-directory", walkfn, kRecursive),
               std::runtime_error);

  WalkDirectory(testdir, walkfn, kRecursive);
  EXPECT_EQ(all_expected_paths, walked_paths);

  walked_paths.clear();

  WalkDirectory(testdir, walkfn, kNotRecursive);
  EXPECT_EQ(first_expected_paths, walked_paths);

  ignition::common::removeAll(testdir);
}

}  // namespace
}  // namespace delphyne
