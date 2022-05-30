// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
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

#include "utility/signal_guard.h"

#include <atomic>
#include <csignal>

namespace delphyne {
namespace common {

std::atomic_bool SignalGuard::allow_signal_guards = true;
std::atomic_bool SignalGuard::allow_signal_handling = true;
std::unordered_map<int, SignalGuard*> SignalGuard::signal_guards{};
std::unordered_map<int, void (*)(int)> SignalGuard::signal_handlers{};

SignalGuard::SignalGuard(int signum, std::function<void()> guard_fn) : signum_(signum), guard_fn_(guard_fn) {
  if (SignalGuard::signal_handlers.count(signum_) == 0) {
    SignalGuard::allow_signal_handling = false;
    SignalGuard::signal_handlers[signum_] = std::signal(signum_, SignalGuard::common_signal_handler);
    SignalGuard::allow_signal_handling = true;
  }

  if (SignalGuard::signal_guards.count(signum_) > 0) {
    prev_signal_guard_ = SignalGuard::signal_guards[signum_];
  }
  SignalGuard::allow_signal_guards = false;
  SignalGuard::signal_guards[signum_] = this;
  SignalGuard::allow_signal_guards = true;
}

SignalGuard::~SignalGuard() {
  SignalGuard::allow_signal_handling = false;
  SignalGuard::signal_guards[signum_] = prev_signal_guard_;
  SignalGuard::allow_signal_handling = true;
}

void SignalGuard::common_signal_handler(int signum) {
  if (SignalGuard::allow_signal_guards) {
    if (SignalGuard::signal_guards.count(signum) > 0) {
      if (SignalGuard::signal_guards[signum]) {
        SignalGuard::signal_guards[signum]->handle();
      }
    }
  }
  if (SignalGuard::allow_signal_handling) {
    if (SignalGuard::signal_handlers[signum]) {
      SignalGuard::signal_handlers[signum](signum);
    }
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
