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

#include "utility/filesystem.h"

#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <ignition/common/Filesystem.hh>

#include "test_utilities/helpers.h"

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
  EXPECT_FALSE(IsValidFilepath(""));
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
  const std::set<std::string> first_expected_paths{ignition::common::joinPaths(testdir, "etc"),
                                                   ignition::common::joinPaths(testdir, "var")};
  const std::set<std::string> all_expected_paths{
      ignition::common::joinPaths(testdir, "etc"), ignition::common::joinPaths(testdir, "etc/init"),
      ignition::common::joinPaths(testdir, "var"), ignition::common::joinPaths(testdir, "var/log"),
      ignition::common::joinPaths(testdir, "var/log/stuff")};
  for (const std::string& path : all_expected_paths) {
    ignition::common::createDirectories(path);
  }

  std::set<std::string> walked_paths{};
  DirectoryWalkFn walkfn = [&walked_paths](const std::string& path) { walked_paths.insert(path); };

  EXPECT_THROW(WalkDirectory("not-a-directory", walkfn, kRecursive), std::runtime_error);

  WalkDirectory(testdir, walkfn, kRecursive);
  EXPECT_EQ(all_expected_paths, walked_paths);

  walked_paths.clear();

  WalkDirectory(testdir, walkfn, kNotRecursive);
  EXPECT_EQ(first_expected_paths, walked_paths);

  ignition::common::removeAll(testdir);
}

}  // namespace
}  // namespace delphyne
