// Copyright 2017 Toyota Research Institute

#pragma once

#include <stdexcept>
#include <string>

namespace delphyne {

// \brief TranslateException is used to signal an error when
// performing a translation between ignition and LCM messages
class TranslateException : public std::runtime_error {
 public:
  /// \brief Creates a new exception
  /// \param[in]  message The message explaining the error
  explicit TranslateException(std::string message)
      : std::runtime_error(message) {}
};

}  // namespace delphyne
