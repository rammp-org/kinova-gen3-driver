#include "kinova_lowlevel/arm_link.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>

namespace kinova {

const char* to_string(LinkState s) {
  switch (s) {
    case LinkState::kConnecting:
      return "connecting";
    case LinkState::kRunning:
      return "running";
    case LinkState::kFaulted:
      return "faulted";
  }
  return "unknown";
}

ArmLink::ArmLink(Transport& t, RtExecutor& exec, LinkListener* listener, ArmLinkConfig cfg)
    : t_(t), exec_(exec), listener_(listener), cfg_(cfg) {
  if (!exec_.config().exit_on_fault) {
    throw std::invalid_argument(
        "ArmLink: the executor must be configured with exit_on_fault, or a faulted arm "
        "would stay in kRunning");
  }
}

void ArmLink::sleep_for(double s, std::atomic<bool>& stop) {
  using clock = std::chrono::steady_clock;
  const auto deadline =
      clock::now() + std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(s));
  for (auto now = clock::now(); !stop.load(std::memory_order_acquire) && now < deadline;
       now = clock::now()) {
    std::this_thread::sleep_for(
        std::min<clock::duration>(std::chrono::milliseconds(10), deadline - now));
  }
}

void ArmLink::set_state(LinkState s, const std::string& reason, bool announce) {
  bool changed;
  {
    std::lock_guard<std::mutex> l(m_);
    changed = announce || st_.state != s;
    st_.state = s;
    st_.reason = reason;
    // A clear asked for in one faulted episode must not fire in the next. One
    // accepted but not yet taken (the link dropped first) resolves here, to this
    // state, so a caller waiting on clears_done still gets an answer. Under m_,
    // as request_clear() is, so a request lands either before this or not at all.
    const bool dropped = s != LinkState::kFaulted && clear_requested_.exchange(false);
    if (clearing_ || dropped) {
      ++st_.clears_done;
      clearing_ = false;
    }
  }
  if (!changed) return;
  if (reason.empty())
    std::fprintf(stderr, "ArmLink: %s\n", to_string(s));
  else
    std::fprintf(stderr, "ArmLink: %s: %s\n", to_string(s), reason.c_str());
  if (listener_) listener_->on_link_state(s);
}

// A second failure while already latched (a clear the arm refused, say) updates
// the reason but is the same fault.
void ArmLink::latch(const std::string& reason) {
  fault_reason_ = reason;
  if (latched_) return;
  latched_ = true;
  std::lock_guard<std::mutex> l(m_);
  ++st_.faults;
}

void ArmLink::note_connect_failure(const std::string& reason) {
  ++connect_failures_;
  // Log the first failure, a changed reason, and every 30th: an arm left off
  // overnight must not fill the log at 1 Hz.
  if (connect_failures_ == 1 || reason != last_connect_error_ || connect_failures_ % 30 == 0) {
    std::fprintf(stderr, "ArmLink: waiting for the arm (attempt %llu): %s\n",
                 static_cast<unsigned long long>(connect_failures_), reason.c_str());
  }
  last_connect_error_ = reason;
  std::lock_guard<std::mutex> l(m_);
  st_.reason = reason;
}

bool ArmLink::request_clear() {
  std::lock_guard<std::mutex> l(m_);
  if (st_.state != LinkState::kFaulted) return false;
  clear_requested_.store(true);
  return true;
}

LinkStatus ArmLink::status() const {
  std::lock_guard<std::mutex> l(m_);
  return st_;
}

// kFaulted: the arm is the base's. Keep feedback flowing (FeedbackTap publishes
// it), watch that the base still answers, and wait for a clear.
ArmLink::Wait ArmLink::wait_faulted(std::atomic<bool>& stop) {
  const auto poll = std::chrono::duration<double>(1.0 / cfg_.fault_poll_hz);
  const auto health = std::chrono::duration<double>(cfg_.health_period_s);
  auto next_health = std::chrono::steady_clock::now();
  int health_failures = 0;
  JointFeedback fb;
  while (!stop.load(std::memory_order_acquire)) {
    if (clear_requested_.exchange(false)) return Wait::kClear;
    // Feedback outside low-level servoing is unverified on the arm; failing to
    // get it only costs published joint state, never the state machine.
    try {
      t_.receive(fb);
    } catch (...) {
    }
    if (std::chrono::steady_clock::now() >= next_health) {
      next_health = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(health);
      try {
        t_.arm_in_fault();
        health_failures = 0;
      } catch (...) {
        if (++health_failures >= cfg_.health_max_failures) return Wait::kLost;
      }
    }
    std::this_thread::sleep_for(poll);
  }
  return Wait::kStop;
}

// The arm takes a moment to leave IN_FAULT after ClearFaults, so poll for up to
// clear_settle_s before calling the fault persistent.
bool ArmLink::fault_persists(std::atomic<bool>& stop) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(cfg_.clear_settle_s));
  while (t_.arm_in_fault()) {
    if (stop.load(std::memory_order_acquire) || std::chrono::steady_clock::now() >= deadline)
      return true;
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / cfg_.fault_poll_hz));
  }
  return false;
}

void ArmLink::run(std::atomic<bool>& stop) {
  set_state(LinkState::kConnecting, "", /*announce=*/true);
  bool connected = false, ever_connected = false;
  bool commanding = false;  // the RT loop ran on this connection
  auto shutdown = [&] {
    try {
      t_.safe_shutdown();
    } catch (...) {
    }
    connected = false;
  };
  // A loss while commanding latches: the arm may come back rebooted under its
  // base, with an operator driving it, so it is not taken back without a clear.
  // A loss before that (an RPC timeout while starting) just retries.
  auto lost = [&](const char* what) {
    if (commanding) latch(std::string("connection lost: ") + what);
    commanding = false;
    set_state(LinkState::kConnecting, what);
    shutdown();
    sleep_for(cfg_.retry_period_s, stop);
  };

  while (!stop.load(std::memory_order_acquire)) {
    if (!connected) {
      try {
        t_.connect();
      } catch (const std::logic_error&) {
        shutdown();
        throw;  // a configuration error: retrying forever would hide it
      } catch (const std::exception& e) {
        note_connect_failure(e.what());
        shutdown();
        sleep_for(cfg_.retry_period_s, stop);
        continue;
      } catch (...) {
        note_connect_failure("unknown exception from connect()");
        shutdown();
        sleep_for(cfg_.retry_period_s, stop);
        continue;
      }
      connected = true;
      connect_failures_ = 0;
      if (ever_connected) {
        std::lock_guard<std::mutex> l(m_);
        ++st_.reconnects;
      }
      ever_connected = true;
    }

    try {
      if (!latched_ && t_.arm_in_fault()) latch("the arm reports a fault");
      if (latched_) {
        set_state(LinkState::kFaulted, fault_reason_);
        t_.release_low_level();
        const Wait w = wait_faulted(stop);
        if (w == Wait::kStop) break;
        if (w == Wait::kLost) throw TransportTimeout("the arm stopped answering while faulted");
        clearing_ = true;
        t_.clear_faults();
        if (fault_persists(stop)) {
          fault_reason_ = "fault persists after clear";
          continue;
        }
        latched_ = false;
      }
      t_.set_servoing_low_level();
      // Known actuator modes before the first command. After a link loss the
      // revert may never have reached the arm, leaving actuators in TORQUE mode,
      // and the first commands of a new run carry zero torque. Redundant on a
      // first start.
      ActuatorModes position;
      position.fill(ActuatorMode::kPosition);
      t_.set_actuator_modes(position);
      set_state(LinkState::kRunning, "");
      commanding = true;
      exec_.run(stop);
      break;  // run() returns only on stop
    } catch (const CommsLost& e) {
      lost(e.what());
    } catch (const TransportTimeout& e) {
      lost(e.what());
    } catch (const std::exception& e) {
      commanding = false;
      latch(e.what());
    } catch (...) {
      commanding = false;
      latch("unknown exception");
    }
  }
  shutdown();
}

}  // namespace kinova
