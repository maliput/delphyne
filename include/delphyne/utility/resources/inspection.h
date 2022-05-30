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
