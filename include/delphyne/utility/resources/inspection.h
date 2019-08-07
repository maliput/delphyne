/**
 * @file include/delphyne/utility/resources/inspection.h
 *
 * Copyright 2018 Toyota Research Institute
 */
/*****************************************************************************
** Includes
****************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ignition/common/URI.hh>

#include "delphyne/macros.h"

#include "delphyne/utility/resources/resources.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace utility {

/*****************************************************************************
** Classes
*****************************************************************************/

/// A singleton for resource reflection.
class ResourceInspector {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(ResourceInspector);

  /// Returns singleton instance.
  static ResourceInspector* Instance();

  /// Associates a resource name extension to a given type.
  /// @param extension Name extension (as in .txt).
  /// @param type Resource type to associate.
  /// @throws std::logic_error if given @p type is null
  /// @throws std::runtime_error if given @p extension is already
  ///                            associated to a type.
  void AssociateExtension(const std::string& extension, const ResourceType* type);

  /// Retrieves a representation of the resource pointed by @p uri
  /// @param uri Identifier of the resource, to be resolved against the
  ///            current Package (see PackageManager class documentation).
  /// @returns Resource representation or nullptr if its type is unknown to
  ///          the inspector.
  /// @throws std::runtime_error if @p uri is not valid.
  /// @throws std::runtime_error if @p uri points to a non-local resource.
  std::unique_ptr<Resource> GetResource(const ignition::common::URI& uri) const;

  /// Extracts the list of resources that the resource pointed by @p uri
  /// depends on (see GetResource()).
  /// @param uri Identifier of the resource to extract dependencies from, to
  ///            be resolved against the current Package (see PackageManager
  ///            class documentation).
  /// @returns The URIs of the dependencies, if any, or an empty vector if
  ///          the resource type is unknown (see AssociateExtension()).
  std::vector<ignition::common::URI> GetDependencies(const ignition::common::URI& uri) const;

 private:
  // Default constructor.
  ResourceInspector();

  // Mapping from name extension to resource type.
  std::map<std::string, const ResourceType*> type_extension_associations_;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace utility
}  // namespace delphyne
