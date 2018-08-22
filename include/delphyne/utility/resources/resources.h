/**
 * @file include/delphyne/utility/resources/resources.h
 *
 * Copyright 2018 Toyota Research Institute
 */
/*****************************************************************************
** Includes
****************************************************************************/

#pragma once

#include <memory>
#include <regex>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <ignition/common/URI.hh>

#include "delphyne/macros.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace utility {

/*****************************************************************************
** Helpers
*****************************************************************************/

namespace internal {

/// A runtime representation of an abstract Type.
/// @tparam Base A class type.
template <class Base>
class Type {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(Type);

  // Forward declaration.
  template <typename ...Args>
  class ConstructibleWith;

  virtual ~Type() = default;

 protected:
  Type() = default;
};

/// A runtime representation of an abstract Type that may be
/// constructible from a given set of parameter types on a
/// derived type.
///
/// @tparam Args Parameters type pack that may be used for
///              construction of a derived type.
template <class Base>
template <typename ...Args>
class Type<Base>::ConstructibleWith : public virtual Type<Base> {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(ConstructibleWith);

  /// Instantiates an object of class Base with the provided @p args.
  /// @param args Parameter pack for construction.
  /// @returns The instantiated Base object.
  virtual std::unique_ptr<Base> Instantiate(Args... args) const = 0;

  virtual ~ConstructibleWith() = default;

 protected:
  ConstructibleWith() = default;
};

/// A runtime representation of a concrete Subtype.
///
/// @tparam Derived Concrete class type that is
///                 a Base class subtype.
/// @tparam Base Abstract class type.
template <class Derived, class Base>
class Subtype : public virtual Type<Base> {
  static_assert(std::is_base_of<Base, Derived>::value, "Not a derived type.");
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(Subtype);

  // Forward declaration.
  template <typename ...Args>
  class ConstructibleWith;

  virtual ~Subtype() = default;

 protected:
  Subtype() = default;
};

/// A runtime representation of a Subtype, that is
/// constructible from a given set of parameter types.
///
/// @tparam Args Parameters type pack for construction of
///              Derived instance. Said instance must be
///              constructible from the provided types.
template <class Derived, class Base>
template <typename ...Args>
class Subtype<Derived, Base>::ConstructibleWith
    : public Type<Base>::template ConstructibleWith<Args...>,
      public Subtype<Derived, Base> {
  static_assert(std::is_constructible<Derived, Args...>::value,
                "Not constructible from given argument types.");
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(ConstructibleWith);

  /// Retrieves singleton type instance.
  static const ConstructibleWith* Instance() {
    static ConstructibleWith instance;
    return &instance;
  }

  std::unique_ptr<Base> Instantiate(Args... args) const override {
    return std::make_unique<Derived>(std::forward<Args>(args)...);
  }

  virtual ~ConstructibleWith() = default;

 protected:
  ConstructibleWith() = default;
};

}  // namespace internal

/*****************************************************************************
** Classes
*****************************************************************************/

/// A class for resource representation.
class Resource {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(Resource);

  /// Constructs a resource associated with the given @p uri.
  explicit Resource(const ignition::common::URI& uri);

  /// Default destructor.
  virtual ~Resource() = default;

  /// Returns this resource URI.
  const ignition::common::URI& Uri() const { return uri_; }

  /// Returns this resource path in the local file system.
  /// @throws std::runtime_error if resolving URI against current
  ///                            Package fails (see PackageManager
  ///                            class documentation).
  /// @throws std::runtime_error if the resource is not local.
  std::string Path() const;

  /// Retrieves the list of resources that this resource depends on.
  /// @returns The URIs of the dependencies, if any.
  virtual std::vector<ignition::common::URI> GetDependencies() const = 0;

 private:
  const ignition::common::URI uri_;
};

/// Type class for a Resource that can be instantiated from a string.
using ResourceType =
    typename internal::Type<Resource>::
    template ConstructibleWith<const ignition::common::URI&>;

/// Type class for a Resource subtype that can be instantiated
/// from a string.
/// @tparam Derived A Resource subclass.
template <typename Derived>
using ResourceSubtype =
    typename internal::Subtype<Derived, Resource>::
    template ConstructibleWith<const ignition::common::URI&>;

/// A simple generic Resource implementation, providing
/// introspection through heavy regex usage.
class GenericResource : public Resource {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(GenericResource);

  /// Constructs generic presentation.
  /// @param uri Identifier associated with the resource.
  /// @param dependency_pattern Regular expression to extract
  ///                           the resource URIs this resource
  ///                           depends on and lists within itself.
  explicit GenericResource(const ignition::common::URI& uri,
                           const std::regex& dependency_pattern);

  std::vector<ignition::common::URI> GetDependencies() const override;

 private:
  // Regular expression to extract resource URIs with.
  const std::regex dependency_pattern_;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace utility
}  // namespace delphyne
