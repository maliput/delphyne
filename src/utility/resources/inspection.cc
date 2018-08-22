// Copyright 2018 Toyota Research Institute

#include "delphyne/utility/resources/inspection.h"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/common/URI.hh>

#include "common/filesystem.h"

#include "delphyne/utility/resources/resources.h"
#include "delphyne/utility/resources/meshes.h"

namespace delphyne {
namespace utility {

ResourceInspector* ResourceInspector::Instance() {
  static ResourceInspector instance;
  return &instance;
}

ResourceInspector::ResourceInspector() {
  AssociateExtension("obj", ResourceSubtype<OBJFile>::Instance());
  AssociateExtension("dae", ResourceSubtype<ColladaFile>::Instance());
}

void ResourceInspector::AssociateExtension(const std::string& extension,
                                           const ResourceType* type) {
  DELPHYNE_VALIDATE(type_extension_associations_.count(extension) == 0,
                    std::runtime_error, "Extension '" + extension + "' already"
                    "asociated.");
  DELPHYNE_VALIDATE(type != nullptr, std::logic_error, "Type is null.");
  type_extension_associations_[extension] = type;
}

std::unique_ptr<Resource>
ResourceInspector::GetResource(const ignition::common::URI& uri) const {
  // Matches resource name extension with known types.
  std::string extension;
  std::tie(std::ignore, extension) = SplitExtension(uri.Path().Str());
  if (type_extension_associations_.count(extension) == 0) {
    // Unknown type, cannot retrieve resource.
    // TODO(hidmic): Should we perform an exhaustive search over
    //               all known resource types?
    return nullptr;
  }
  // Retrieve resource using the proper type.
  const ResourceType* resource_type =
      type_extension_associations_.at(extension);
  return resource_type->Instantiate(uri);
}

std::vector<ignition::common::URI>
ResourceInspector::GetDependencies(const ignition::common::URI& uri) const {
  std::unique_ptr<Resource> resource = GetResource(uri);
  if (resource == nullptr) return {};
  return resource->GetDependencies();
}

}  // namespace utility
}  // namespace delphyne
