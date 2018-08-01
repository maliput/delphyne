// Copyright 2018 Toyota Research Institute

#include "common/compression.h"

#include <cstdlib>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include <ignition/common/Filesystem.hh>

#include "helpers.h"

namespace delphyne {
namespace {

class ZipTest : public test::TestWithFiles {
 protected:
  void DoSetUp() override {
    using ignition::common::joinPaths;
    initial_path_ = joinPaths(tmpdir(), "initial_stuff");
    archive_path_ = joinPaths(tmpdir(), "archive.zip");
    final_path_ = joinPaths(tmpdir(), "final_stuff");
    const std::string path_to_nested_dir =
        joinPaths(initial_path_, "deeply/nested/dir/");
    ignition::common::createDirectories(path_to_nested_dir);
    std::ofstream nested_file_fs(joinPaths(
        path_to_nested_dir, "nested.txt"));
    nested_file_fs << "dummy content";
    std::ofstream file_fs(joinPaths(initial_path_, "test.txt"));
    file_fs << "more dummy content";
    ignition::common::createDirectories(final_path_);
  }

  bool Zip(const std::string& source_path,
           const std::string& destination_path) {
    const std::string command =
        "cd " + source_path + "; zip -r " +
        destination_path + " * 1>/dev/null 2>&1";
    return (std::system(command.c_str()) == 0);
  }

  bool Unzip(const std::string& archive_path,
             const std::string& extract_path) {
    const std::string command =
        "unzip -o -q " + archive_path + " -d " +
        extract_path + " 1>/dev/null 2>&1";
    return (std::system(command.c_str()) == 0);
  }

  bool Diff(const std::string& dir_a, const std::string& dir_b) {
    const std::string command =
        "diff -rq " + dir_a + " " + dir_b + " 1>/dev/null 2>&1";
    return (std::system(command.c_str()) != 0);
  }

  std::string initial_path_{""};
  std::string archive_path_{""};
  std::string final_path_{""};
};

TEST_F(ZipTest, Compression) {
  ZipDirectory(initial_path_, archive_path_);
  ASSERT_TRUE(Unzip(archive_path_, final_path_));
  EXPECT_FALSE(Diff(initial_path_, final_path_));
}

TEST_F(ZipTest, Decompression) {
  ASSERT_TRUE(Zip(initial_path_, archive_path_));
  UnzipDirectory(archive_path_, final_path_);
  EXPECT_FALSE(Diff(initial_path_, final_path_));
}


}  // namespace
}  // namespace delphyne
