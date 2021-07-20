// Copyright 2018 Toyota Research Institute

#include "utility/compression.h"

#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>
#include <zip.h>

#include "utility/filesystem.h"

namespace delphyne {

// TODO(hidmic): Find a proper cross-platform mechanism to zip a
//               directory recursively.
void ZipDirectory(const std::string& source_path, const std::string& destination_path) {
  if (!ignition::common::isDirectory(source_path)) {
    throw std::runtime_error(
        "Given source path is not"
        " an existing directory.");
  }
  if (ignition::common::exists(destination_path)) {
    throw std::runtime_error(
        "Given destination path"
        " already exists.");
  }
  // Zips `source_path` contents recursively and output to `destination_path`.
  int errorp = 0;
  auto zip_deleter = [](zip_t* zipper) {
    if (zip_close(zipper) < 0) {
      throw std::runtime_error("Failed to close zip: " + std::string(zip_strerror(zipper)));
    }
  };
  std::unique_ptr<zip_t, decltype(zip_deleter)> zipper(zip_open(destination_path.c_str(), ZIP_CREATE, &errorp),
                                                       zip_deleter);
  if (zipper == nullptr) {
    zip_error_t ziperror;
    zip_error_init_with_code(&ziperror, errorp);
    throw std::runtime_error("Failed to open destination archive " + destination_path + ": " +
                             zip_error_strerror(&ziperror));
  }
  constexpr const bool kRecursive = true;
  constexpr const int kZeroOffset = 0;
  constexpr const int kFullLength = 0;
  DirectoryWalkFn walkfn = [&zipper, &source_path](const std::string& path) {
    const std::string basename = path.substr(source_path.length() + 1);
    if (ignition::common::isDirectory(path)) {
      if (zip_dir_add(zipper.get(), basename.c_str(), ZIP_FL_ENC_UTF_8) < 0) {
        throw std::runtime_error("Failed to add directory to zip: " + std::string(zip_strerror(zipper.get())));
      }
    } else {
      // Ownership for zip_source_t instances is passed to libzip
      // upon successful zip_file_add() call.
      zip_source_t* source = zip_source_file(zipper.get(), path.c_str(), kZeroOffset, kFullLength);
      if (source == nullptr) {
        throw std::runtime_error("Failed to add file to zip: " + std::string(zip_strerror(zipper.get())));
      }
      if (zip_file_add(zipper.get(), basename.c_str(), source, ZIP_FL_ENC_UTF_8) < 0) {
        zip_source_free(source);
        throw std::runtime_error("Failed to add file to zip: " + std::string(zip_strerror(zipper.get())));
      }
    }
  };
  WalkDirectory(source_path, walkfn, kRecursive);
}

// TODO(hidmic): Find a proper cross-platform mechanism to unzip.
void UnzipDirectory(const std::string& archive_path, const std::string& extract_path) {
  if (!ignition::common::isFile(archive_path)) {
    throw std::runtime_error(
        "Given source path is not"
        " an existing directory.");
  }
  if (!ignition::common::isDirectory(extract_path)) {
    throw std::runtime_error(
        "Given extraction path must"
        " be an existing directory.");
  }
  int errorp = 0;
  auto zip_deleter = [](zip_t* zipper) { zip_close(zipper); };
  std::unique_ptr<zip_t, decltype(zip_deleter)> zipper(zip_open(archive_path.c_str(), ZIP_RDONLY, &errorp),
                                                       zip_deleter);
  if (zipper == nullptr) {
    zip_error_t ziperror;
    zip_error_init_with_code(&ziperror, errorp);
    throw std::runtime_error("Failed to open archive " + archive_path + ": " + zip_error_strerror(&ziperror));
  }
  char buffer[4096];
  constexpr const int kNoZipFlags = 0;
  auto zip_file_deleter = [](zip_file_t* file) { zip_fclose(file); };
  for (int i = 0; i < zip_get_num_entries(zipper.get(), 0); ++i) {
    zip_stat_t stat;
    if (zip_stat_index(zipper.get(), i, kNoZipFlags, &stat) < 0) {
      throw std::runtime_error("Failed to stat archive entry: " + std::string(zip_strerror(zipper.get())));
    }
    if (!(stat.valid & ZIP_STAT_NAME && stat.valid & ZIP_STAT_SIZE)) {
      throw std::runtime_error("Archive entry stats are invalid: " + std::to_string(stat.valid));
    }
    const std::string path = ignition::common::joinPaths(extract_path, stat.name);
    if (path.back() != '/') {
      std::unique_ptr<zip_file_t, decltype(zip_file_deleter)> ifile(zip_fopen_index(zipper.get(), i, kNoZipFlags),
                                                                    zip_file_deleter);
      std::ofstream ofile(path);
      int total_bytes_read = 0;
      int total_bytes_to_read = stat.size;
      while (total_bytes_read < total_bytes_to_read) {
        int bytes_read = zip_fread(ifile.get(), buffer, sizeof(buffer));
        if (bytes_read < 0) {
          throw std::runtime_error("Failed to read from archive entry: " + std::string(zip_strerror(zipper.get())));
        }
        ofile.write(buffer, bytes_read);
        total_bytes_read += bytes_read;
      }
    } else {
      ignition::common::createDirectory(path);
    }
  }
}

}  // namespace delphyne
