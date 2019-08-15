// Copyright 2019 Toyota Research Institute

#include "utility/signal_guard.h"

#include <atomic>
#include <csignal>

namespace delphyne {
namespace common {

std::atomic_bool SignalGuard::allow_signal_handling = false;
std::unordered_map<int, SignalGuard*> SignalGuard::signal_guards{};
std::unordered_map<int, void(*)(int)> SignalGuard::signal_handlers{};

SignalGuard::SignalGuard(int signum, std::function<void()> guard_fn)
  : signum_(signum), guard_fn_(guard_fn) {
  SignalGuard::allow_signal_handling = false;
  if (SignalGuard::signal_handlers.count(signum_) == 0) {
    SignalGuard::signal_handlers[signum_] =
      std::signal(signum_, SignalGuard::common_signal_handler);
  }
  if (SignalGuard::signal_guards.count(signum_) > 0) {
    prev_signal_guard_ = SignalGuard::signal_guards[signum_];
  }
  SignalGuard::signal_guards[signum_] = this;
  SignalGuard::allow_signal_handling = true;
}

SignalGuard::~SignalGuard() {
  SignalGuard::allow_signal_handling = false;
  SignalGuard::signal_guards[signum_] = prev_signal_guard_;
  SignalGuard::allow_signal_handling = true;
}

void SignalGuard::common_signal_handler(int signum) {
  if (SignalGuard::allow_signal_handling) {
    if (SignalGuard::signal_guards.count(signum) > 0) {
      if (SignalGuard::signal_guards[signum]) {
        SignalGuard::signal_guards[signum]->handle();
      }
    }
    SignalGuard::signal_handlers[signum](signum);
  }
}

void SignalGuard::handle() {
  if (prev_signal_guard_) {
    prev_signal_guard_->handle();
  }
  guard_fn_();
}

}  // namespace common
}  // namespace delphyne

