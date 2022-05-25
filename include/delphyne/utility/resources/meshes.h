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
