/**
 * @file include/delphyne/utility/package.h
 *
 * Copyright 2018 Toyota Research Institute
 */
/*****************************************************************************
** Includes
****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ignition/common/URI.hh>

#include "delphyne/macros.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace utility {

/*****************************************************************************
** Functions
*****************************************************************************/

/// Returns given @p uri_or_path as a URI.
///
/// Absolute paths are associated with the 'file://' scheme. Relative
/// paths are associated with a 'package://' scheme. Any other URI
/// passes through unchanged.
ignition::common::URI ToURI(const std::string& uri_or_path);

/*****************************************************************************
** Classes
*****************************************************************************/

/// A class for resource management.
class Package {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(Package);

  /// @brief Checks whether the local file referred by
  /// @p uri_or_path exists or not.
  bool Exists(const std::string& uri_or_path) const {
    return (this->Find(uri_or_path) != "");
  }

  /// @brief Checks whether the local file referred by
  /// @p uri exists or not.
  bool Exists(const ignition::common::URI& uri) const {
    return (this->Find(uri) != "");
  }

  /// @brief Finds local file by @p uri_or_path.
  /// @see Find(const ignition::common::URI&)
  /// @param[in] uri The path of the file.
  /// @returns The full absolute path to the file, or an empty string
  ///          if none could be found
  std::string Find(const std::string& uri_or_path) const {
    return this->Find(ToURI(uri_or_path));
  }

  /// @brief Finds local file by @p uri.
  ///
  /// @param[in] uri The location of the file.
  /// @returns The full absolute path to the file, or an empty string
  ///          if none could be found
  std::string Find(const ignition::common::URI& uri) const {
    if (!uri.Valid()) return "";
    return this->DoFind(uri);
  }

 protected:
  Package() = default;

 private:
  // @see Find(const ignition::common::URI&)
  virtual std::string DoFind(const ignition::common::URI& uri) const = 0;
};

/// A Package subclass that leverages a Delphyne system installation.
///
/// Currently supported URI schemes are 'file://' and 'package://',
/// the latter performing a lookup throughout the directories specified
/// in the $DELPHYNE_PACKAGE_PATH environment variable.
class SystemPackage : public Package {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemPackage);

  SystemPackage();

 private:
  std::string DoFind(const ignition::common::URI& uri) const override;

  // List of paths that can contain resources (e.g. meshes).
  // The content of this variable is populated with the value of the
  // DELPHYNE_PACKAGE_PATH environment variable.
  std::vector<std::string> package_paths_{};
};

/// A Package subclass that bundles all resources
/// in a self contained directory structure, suitable
/// for transport.
class BundledPackage : public Package {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(BundledPackage);

  /// Associates instance with the given @p path.
  ///
  /// Given @p path need not exist at construction time.
  explicit BundledPackage(const std::string& path);

  /// Adds local resource referred by @p uri_or_path and
  /// located at @p path to the bundle.
  /// @see Add(const ignition::common::URI&, const std::string&)
  bool Add(const std::string& uri, const std::string& path) {
    return this->Add(ToURI(uri), path);
  }

  /// Adds local resource referred by @p uri and
  /// located at @p path to the bundle.
  /// @param[in] uri Resource identifier.
  /// @param[in] path Resource location in the filesystem.
  /// @returns true if the resource was succesfully added,
  ///          false otherwise.
  bool Add(const ignition::common::URI& uri,
           const std::string& path);

 private:
  // Resolves the given @p uri into an internal location suitable
  // for storage.
  // @param[in] uri Resource identifier.
  // @returns A full path into the bundle.
  std::string ResolveToInternalLocation(const ignition::common::URI& uri) const;

  std::string DoFind(const ignition::common::URI& uri) const override;

  // Package bundle root path.
  std::string path_{};
};

/// A singleton to manage package usage during runtime.
class PackageManager {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(PackageManager);

  static PackageManager* Instance();

  /// Takes and uses given @p package
  void Use(std::unique_ptr<Package> package);

  /// Returns the current package in use.
  const Package& package_in_use() const;

 private:
  // Default constructor.
  PackageManager() = default;

  // Package instance in use.
  mutable std::unique_ptr<Package> package_{nullptr};
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace utility
}  // namespace delphyne
