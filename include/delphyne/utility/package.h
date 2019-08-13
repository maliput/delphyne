/**
 * @file include/delphyne/utility/package.h
 *
 * Copyright 2018 Toyota Research Institute
 */
/*****************************************************************************
** Includes
****************************************************************************/

#pragma once

#include <cstring>
#include <iostream>

#include <memory>
#include <string>
#include <vector>

#include <ignition/common/URI.hh>

#include "delphyne/macros.h"
#include "delphyne/utility/resources/inspection.h"

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

  /// Resolves a @p uri_or_path, specific to this package, into a
  /// full regular URI.
  ///
  /// @see ToURI(const std::string&)
  /// @see Resolve(const ignition::common::URI&)
  ignition::common::URI Resolve(const std::string& uri_or_path) const { return this->Resolve(ToURI(uri_or_path)); }

  /// Resolves a @p uri, specific to this package, into a full regular URI.
  ///
  /// @param[in] uri Package specific resource identifier to resolve.
  /// @returns A valid URI on success, an invalid one on failure.
  /// @throws std::runtime_error if @p uri is not valid (i.e. uri.Valid()
  ///                            is false).
  ignition::common::URI Resolve(const ignition::common::URI& uri) const {
    DELPHYNE_VALIDATE(uri.Valid(), std::runtime_error, uri.Str() + " is not a valid URI.");
    return this->DoResolve(uri);
  }

 protected:
  Package() = default;

 private:
  // @see Resolve(const ignition::common::URI&)
  virtual ignition::common::URI DoResolve(const ignition::common::URI& uri) const = 0;
};

/// A Package subclass that leverages a Delphyne system installation.
///
/// Currently supported URI schemes are 'file://' and 'package://',
/// the latter performing a lookup throughout the directories specified
/// in the $DELPHYNE_PACKAGE_PATH environment variable.
class SystemPackage : public Package {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemPackage);

  /// Default constructor.
  /// @throws std::runtime_error if DELPHYNE_PACKAGE_PATH environment
  ///                            variable is not set.
  SystemPackage();

 private:
  ignition::common::URI DoResolve(const ignition::common::URI& uri) const override;

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

  /// Adds @b local resource referred by @p uri_or_path.
  /// @see Add(const ignition::common::URI&, const std::string&)
  void Add(const std::string& uri_or_path) { this->Add(ToURI(uri_or_path)); }

  /// Adds @b local resource referred by @p uri.
  /// @param[in] uri Identifier of the resource to be added.
  /// @throws std::runtime_error if given @p uri is not valid
  ///                            (i.e. uri.Valid() is false).
  /// @throws std::runtime_error if the resource is not local. Only
  ///                            file and package schemes are currently
  ///                            supported.
  /// @throws std::runtime_error if the resource or any of its dependencies
  ///                            could not be found in the current package.
  ///                            See PackageManager and ResourceInspector
  ///                            singletons.
  void Add(const ignition::common::URI& uri) {
    const std::vector<ignition::common::URI> dep_uris = ResourceInspector::Instance()->GetDependencies(uri);
    for (const auto& dep_uri : dep_uris) {
      if (!this->Resolve(dep_uri).Valid()) {
        this->Add(dep_uri);
      }
    }
    this->DoAdd(uri);
  }

 private:
  // Resolves the given @p uri into an internal location suitable
  // for storage.
  // @param[in] uri Resource identifier.
  // @returns A full path into the bundle.
  std::string ResolveToInternalPath(const ignition::common::URI& uri) const;

  // @see Add(const ignition::common::URI&)
  virtual void DoAdd(const ignition::common::URI& uri);

  ignition::common::URI DoResolve(const ignition::common::URI& uri) const override;

  // Package bundle root path.
  std::string path_{};
};

/// A singleton to manage package usage during runtime.
class PackageManager {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(PackageManager);

  static PackageManager* Instance();

  /// Takes and uses given @p package.
  /// @warning This call invalidates references returned
  ///          by PackageManager::package_in_use().
  void Use(std::unique_ptr<Package> package);

  /// Returns the an immutable reference to the current
  /// package in use.
  const Package& package_in_use() const;

 private:
  PackageManager() = default;

  // Package instance in use.
  mutable std::unique_ptr<Package> package_{nullptr};
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace utility
}  // namespace delphyne
