// Copyright 2019 Toyota Research Institute

#pragma once

#include <atomic>
#include <functional>
#include <unordered_map>

#include "delphyne/macros.h"

namespace delphyne {
namespace common {

class SignalGuard {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(SignalGuard);

  SignalGuard(int signum, std::function<void()> guard_fn);
  ~SignalGuard();

 private:
  static std::atomic_bool allow_signal_guards;
  static std::atomic_bool allow_signal_handling;
  static std::unordered_map<int, SignalGuard*> signal_guards;
  static std::unordered_map<int, void (*)(int)> signal_handlers;
  static void common_signal_handler(int signum);

  void handle();

  int signum_;
  std::function<void()> guard_fn_;
  SignalGuard* prev_signal_guard_{nullptr};
};

}  // namespace common
}  // namespace delphyne
