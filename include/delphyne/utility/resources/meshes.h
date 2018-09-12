/**
 * @file include/delphyne/utility/resources/meshes.h
 *
 * Copyright 2018 Toyota Research Institute
 */
/*****************************************************************************
** Includes
****************************************************************************/

#pragma once

#include <regex>
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

/// A Resource representation of an OBJ mesh file.
class OBJFile : public GenericResource {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(OBJFile);

  /// Constructs representation for the OBJ mesh
  /// file associated with the given @p uri.
  /// @param uri Identifier of the OBJ mesh.
  explicit OBJFile(const ignition::common::URI& uri);
};

/// A Resource representation of a Collada mesh file.
// TODO(hidmic): Consider using a parser instead of
//               plain regular expressions.
class ColladaFile : public GenericResource {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(ColladaFile);

  /// Constructs representation for the Collada mesh
  /// file associated with the given @p uri.
  /// @param uri Identifier of the Collada mesh.
  explicit ColladaFile(const ignition::common::URI& uri);
};

/// A Resource representation of an MTL material file as
/// found in OBJ files.
class MTLFile : public GenericResource {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(MTLFile);

  /// Constructs representation for the MTL material
  /// file associated with the given @p uri.
  /// @param uri Identifier of the MTL material.
  explicit MTLFile(const ignition::common::URI& uri);
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace utility
}  // namespace delphyne
