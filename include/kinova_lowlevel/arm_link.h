#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/transport.h"
namespace kinova {

// The link to the arm, as the rest of the driver sees it.
//   kConnecting: no session with the arm; connect() is retried until it answers.
//   kRunning:    low-level servoing, the RT loop is commanding.
//   kFaulted:    a fault is latched and the arm is handed back to its base
//                (actuators POSITION, single-level servoing) until an explicit
//                request_clear().
enum class LinkState { kConnecting, kRunning, kFaulted };
const char* to_string(LinkState s);

struct LinkStatus {
  LinkState state = LinkState::kConnecting;
  // Why the link is not running: the fault, or the last failed connect attempt.
  // Empty while kRunning.
  std::string reason;
  uint64_t faults = 0;      // faults latched since start
  uint64_t reconnects = 0;  // successful connects after the first
  // Clear attempts that have resolved: bumped, together with the state/reason it
  // resolved to, when the state machine next publishes after taking a clear. A
  // caller that read it before request_clear() waits for it to move to learn the
  // outcome (kRunning, or kFaulted with the reason).
  uint64_t clears_done = 0;
};

// Told about every state change, on ArmLink::run()'s thread, before the new
// state takes effect on the arm (before run() starts commanding for kRunning).
class LinkListener {
 public:
  virtual ~LinkListener() = default;
  virtual void on_link_state(LinkState s) = 0;
};

struct ArmLinkConfig {
  double retry_period_s = 1.0;   // between connect attempts, and after a link loss
  double fault_poll_hz = 100.0;  // receive() while kFaulted, so feedback stays live
  double health_period_s = 0.5;  // arm_in_fault() while kFaulted: is the arm still there?
  int health_max_failures = 3;   // that many unanswered in a row = link lost
};

// Owns the link lifecycle so that no arm-side event ends the process: a fault,
// the arm forced out of low-level, a disconnect, an arm that is off at start, an
// RPC timeout, or any exception out of the RT loop. It runs on the caller's
// thread, the one that would otherwise call RtExecutor::run() directly, and calls
// run() while kRunning: the executor stays the only thread-aware unit, and
// connect, backoff, revert and fault polling all happen outside the RT loop.
//
//   arm unreachable  -> retry every retry_period_s, forever
//   comms lost       -> revert + tear down, reconnect; no clear needed, and the
//                       arm resumes holding where it is (the listener re-requests
//                       a hold mode; ArmLink itself never picks a mode)
//   fault, rejected command, unknown exception
//                    -> hand the arm back to its base, report, wait for
//                       request_clear(); a fault stays latched across reconnects
//
// The fault is never cleared on ArmLink's own initiative.
class ArmLink {
 public:
  // Throws std::invalid_argument unless exec.config().exit_on_fault: without it
  // a faulted arm would stay in kRunning. `listener` may be null.
  ArmLink(Transport& t, RtExecutor& exec, LinkListener* listener, ArmLinkConfig cfg = {});

  // Blocks until `stop`, then returns with the transport shut down. Lets only
  // std::logic_error out (an actuator-count mismatch from connect(): a
  // configuration error retrying would hide).
  void run(std::atomic<bool>& stop);

  // Any thread. Asks run() to clear the fault and re-enter low-level. True only
  // while kFaulted; the clear itself happens on run()'s thread.
  bool request_clear();

  // Any thread.
  LinkStatus status() const;

 private:
  enum class Wait { kClear, kLost, kStop };
  Wait wait_faulted(std::atomic<bool>& stop);
  // Logs and tells the listener on a change, or always with `announce`.
  void set_state(LinkState s, const std::string& reason, bool announce = false);
  void latch(const std::string& reason);
  void note_connect_failure(const std::string& reason);
  static void sleep_for(double s, std::atomic<bool>& stop);

  Transport& t_;
  RtExecutor& exec_;
  LinkListener* listener_;
  ArmLinkConfig cfg_;

  bool latched_ = false;      // run()'s thread only
  bool clearing_ = false;     // run()'s thread only: a clear taken, not yet resolved
  std::string fault_reason_;  // run()'s thread only; survives a reconnect
  uint64_t connect_failures_ = 0;
  std::string last_connect_error_;
  std::atomic<bool> clear_requested_{false};
  mutable std::mutex m_;
  LinkStatus st_;  // guarded by m_
};

}  // namespace kinova
