// Copyright 2017 Toyota Research Institute

#pragma once

#include <iterator>
#include <type_traits>
#include <utility>
#include <vector>

#include <pybind11/cast.h>
#include <pybind11/pybind11.h>

namespace pybind11 {
namespace detail {

template <typename Source, typename OutputIt,
          typename std::enable_if<
            !std::is_pointer<Source>::value, int>::type = 0>
void collect_references(const Source& source, OutputIt output) { }

template <typename Source, typename OutputIt,
          typename std::enable_if<
            std::is_pointer<Source>::value, int>::type = 0>
void collect_references(const Source& source, OutputIt output) {
  *output++ = cast(source);
}

template <typename Element, typename OutputIt>
void collect_references(const std::vector<Element>& source, OutputIt output) {
  for (const Element& e : source) {
    collect_references(e, output);
  }
}

template <typename Element, typename OutputIt>
void collect_references(const std::pair<Element, Element>& source,
                        OutputIt output) {
  collect_references(source.first, output);
  collect_references(source.second, output);
}

}  // namespace detail

template <typename Return, typename Class, typename ... Args>
auto guard_internal_references(Return (Class::*method)(Args...)) {
  return [method] (Class *that, Args... args) {
    Return return_value = (that->*method)(std::forward<Args>(args)...);
    std::vector<object> visibilized;
    detail::collect_references(
        return_value, std::back_inserter(visibilized));
    object self = cast(that);
    for (object& obj : visibilized) {
      keep_alive_impl(obj, self);
    }
    return cast(return_value);
  };
}

template <typename Return, typename Class, typename ... Args>
auto guard_internal_references(Return (Class::*method)(Args...) const) {
  return [method] (Class *that, Args... args) {
    Return return_value = (that->*method)(std::forward<Args>(args)...);
    std::vector<object> visibilized;
    detail::collect_references(
        return_value, std::back_inserter(visibilized));
    object self = cast(that);
    for (object& obj : visibilized) {
      keep_alive_impl(obj, self);
    }
    return cast(return_value);
  };
}

}  // namespace pybind11
