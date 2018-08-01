/**
 * @file include/delphyne/utility/resources.h
 *
 * Copyright 2018 Toyota Research Institute
 */
/*****************************************************************************
** Includes
****************************************************************************/

#pragma once

#include <vector>

#include <ignition/common/URI.hh>

#include "delphyne/macros.h"

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

  static ResourceInspector* Instance();

  /// Extracts the list of resources that the resource pointed by @p uri
  /// depends on.
  /// @param uri Identifier for the resource to extract dependencies from.
  /// @returns The URIs of the resource dependencies, if any.
  std::vector<ignition::common::URI>
  Depends(const ignition::common::URI& uri) const;

 private:
  ResourceInspector() = default;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace utility
}  // namespace delphyne
