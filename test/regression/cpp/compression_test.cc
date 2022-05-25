// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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

#include "utility/compression.h"

#include <cstdlib>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <ignition/common/Filesystem.hh>

#include "test_utilities/helpers.h"

namespace delphyne {
namespace {

class ZipTest : public test::TestWithFiles {
 protected:
  void DoSetUp() override {
    using ignition::common::joinPaths;
    initial_path_ = joinPaths(tmpdir(), "initial_stuff");
    archive_path_ = joinPaths(tmpdir(), "archive.zip");
    final_path_ = joinPaths(tmpdir(), "final_stuff");
    const std::string path_to_nested_dir = joinPaths(initial_path_, "deeply/nested/dir/");
    ignition::common::createDirectories(path_to_nested_dir);
    std::ofstream nested_file_fs(joinPaths(path_to_nested_dir, "nested.txt"));
    nested_file_fs << "dummy content";
    std::ofstream file_fs(joinPaths(initial_path_, "test.txt"));
    file_fs << "more dummy content";
    ignition::common::createDirectories(final_path_);
  }

  bool Zip(const std::string& source_path, const std::string& destination_path) {
    const std::string command = "cd " + source_path + "; zip -r " + destination_path + " * 1>/dev/null 2>&1";
    return (std::system(command.c_str()) == 0);
  }

  bool Unzip(const std::string& archive_path, const std::string& extract_path) {
    const std::string command = "unzip -o -q " + archive_path + " -d " + extract_path + " 1>/dev/null 2>&1";
    return (std::system(command.c_str()) == 0);
  }

  bool Diff(const std::string& dir_a, const std::string& dir_b) {
    const std::string command = "diff -rq " + dir_a + " " + dir_b + " 1>/dev/null 2>&1";
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
