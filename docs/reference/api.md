# API Reference

Everything lives in namespace `kinova`. Units are **SI / radians** throughout;
KORTEX's degrees/N·m conversions happen only inside the transport. Methods marked
**RT-safe** perform no heap allocation, locking, or blocking after construction
and are safe to call from the 1 kHz control thread.

This is the lookup reference. For narrative explanations see the
[Control Modes guide](../guide/control-modes.md); for the impedance math see the
[Deep Dive](../deep-dive/impedance.md).

---

## Value types

### `joint_types.h`

```cpp
inline constexpr int kNumJoints = 7;
using JointVec = Eigen::Matrix<double, 7, 1>;       // SI: rad, rad/s, or N·m

enum class ActuatorMode : uint8_t { kPosition, kVelocity, kTorque, kCurrent };

struct JointFeedback {
  JointVec q, qd, tau, current;     // position, velocity, torque, current
  uint64_t frame_id = 0;
  bool     fault = false;
};

struct JointCommand {
  ActuatorMode mode = ActuatorMode::kTorque;
  JointVec position, velocity, torque;
};
```

Fixed-size, allocation-free POD value types shared by every unit. No KORTEX or
Pinocchio types leak through them.

### `cartesian_types.h`

```cpp
using Vector6   = Eigen::Matrix<double, 6, 1>;      // spatial: [vx vy vz | wx wy wz]
using Jacobian6 = Eigen::Matrix<double, 6, 7>;      // 6×7 frame Jacobian
struct Pose { Eigen::Vector3d p; Eigen::Quaterniond R; };   // SE(3); p [m], R orientation
```

`Pose` defaults to the identity (zero translation, identity rotation). Fixed-size,
RT-safe to copy.

### `units.h`

```cpp
JointVec deg_to_rad(const JointVec& d);
JointVec rad_to_deg(const JointVec& r);
double   wrap_to_pi(double a);          // wrap to (-π, π]
```

---

## `Dynamics` — `dynamics.h`

The only unit that includes Pinocchio. Loads the URDF once and preallocates; the
three query methods are RT-safe.

```cpp
explicit Dynamics(const std::string& urdf_path,
                  const std::string& ee_frame = "gen3_end_effector_link");

void gravity(const JointVec& q, JointVec& tau_out);   // RT-safe: generalized gravity torque
Pose fk(const JointVec& q);                            // RT-safe: pose of ee_frame (world frame)
void jacobian(const JointVec& q, Jacobian6& J_out);    // RT-safe: 6×7, LOCAL_WORLD_ALIGNED
int  nv() const;   // # velocity DOF (must be 7)
int  nq() const;   // # config coords (>7: continuous joints packed as cos/sin)
```

- **Constructor throws** (`std::runtime_error`) if the URDF's `nv != 7` or if
  `ee_frame` doesn't exist — fail loud at startup rather than silently mis-control.
- **`jacobian`** uses Pinocchio's `LOCAL_WORLD_ALIGNED` reference frame: spatial
  velocity in world-aligned axes at the EE origin. `J_out` is filled in place
  (caller owns the fixed-size storage).
- See the [Deep Dive](../deep-dive/impedance.md#frames) on the frame convention.

### `pose_error` — `cartesian.h`

```cpp
Vector6 pose_error(const Pose& desired, const Pose& current);
//   returns [ p_d − p ;  rotvec(R_d · R⁻¹) ]   — decoupled geometric SE(3) error
```

Eigen-only (no Pinocchio). Position error in the world frame; orientation error as
a shortest-path rotation vector. Singularity-free for orientation errors below π.

### `ee_wrench_from_residual` — `cartesian.h`

```cpp
Vector6 ee_wrench_from_residual(const Jacobian6& J, const JointVec& tau_ext,
                                double damping = 0.05);
//   returns (J·Jᵀ + λ²I)⁻¹ J · τ_ext   — damped pseudoinverse of Jᵀ
```

Eigen-only. Maps an external joint-torque residual to the EE wrench that
explains it, with a hard amplification cap of `1/(2λ)` near singular poses. The
Supervisor's pump uses it to fill `ArmState::ee_wrench` with `τ_ext = g(q) − τ`
(the normalized convention — see `JointFeedback::tau`); NaN under a fault.
Derivation, accuracy floor, and the λ trade:
`docs/superpowers/specs/2026-10-04-ee-wrench-design.md`.

---

## Control modes

### `ControlMode` (interface) — `control_mode.h`

```cpp
class ControlMode {
 public:
  virtual ActuatorModes required_modes() const = 0;
  virtual void on_enter(const JointFeedback&) = 0;
  virtual void compute(const JointFeedback& fb, double dt_s, JointCommand& out) = 0;  // RT-safe
  virtual void on_exit() = 0;
};
using ActuatorModes = std::array<ActuatorMode, 7>;
```

Implement this to add a control law. `compute()` must be RT-safe.

### `JointTorqueMode` — `joint_torque_mode.h`

```cpp
struct JointTorqueParams {
  double scale         = 1.0;   // fraction of gravity to apply (0.5 = gentle sag)
  double damping       = 0.0;   // joint-velocity damping (N·m·s/rad)
  // Per-joint ceiling on the TOTAL output. Joints 5-7 have a URDF effort limit
  // of 9 N·m; a scalar sized for the proximal joints would overrun the wrist
  // by 4x. Mirrors JointImpedanceParams::torque_limit.
  JointVec torque_limit = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();
  double cmd_timeout_s = 0.1;   // staleness watchdog window; <=0 disables
  double cmd_decay_s   = 0.05;  // ramp tau_ff -> 0 over this window on staleness
                                 // (<=0 => hard zero)
};
JointTorqueMode(Dynamics& dyn, JointTorqueParams p = {});

// Non-RT: call from a single supervisor thread. Lock-free publish (two-buffer +
// atomic index); compute() reads once per cycle.
void set_torque(const JointVec& tau_ff) noexcept;

// Non-RT: re-arm the staleness watchdog. s >= 0 arms with s; s < 0 restores
// this mode's own configured default (cmd_timeout_s) -- see Streaming below.
void set_command_timeout(double s) noexcept;
```

Law: `tau = scale·gravity(q) − damping·q̇ + tau_ff`, clamped per joint. Sets
`command.position = q` (passthrough). If `tau_ff` goes stale for longer than
`cmd_timeout_s`, a watchdog ramps it to zero over `cmd_decay_s` and the mode
reverts to gravity-compensation hold. **With `tau_ff` never set, this mode
*is* gravity compensation** — `benchmark_grav_comp` runs exactly this
zero-feedforward path. See the
[guide](../guide/control-modes.md#gravity-compensation-jointtorquemode).

### `FloatMode` — `float_mode.h`

The contact-stop control law: what the arm switches to when the contact detector
trips. It yields to any push, slows under joint damping, and stays where it is
left; a weak spring on a dragged anchor resists only model-error creep.

```cpp
struct FloatParams {
  JointVec damping = (JointVec() << 5, 5, 5, 5, 1.5, 1.5, 1.5).finished();          // N·m·s/rad
  JointVec spring  = (JointVec() << 28.6, 28.6, 28.6, 28.6, 8.6, 8.6, 8.6).finished(); // N·m/rad
  double drag_band_rad = 2.0 * kDeg2Rad;   // anchor trails q by at most this
  JointVec torque_limit = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();
};
FloatMode(Dynamics& dyn, FloatParams p = {});  // throws std::invalid_argument
const JointVec& anchor() const noexcept;       // current dragged anchor q_a
```

Law: `tau = gravity(q) − D·q̇ − K_s·(q − q_a)`, clamped per joint to
`torque_limit`. `on_enter` sets `q_a = fb.q`. Each cycle, per joint, if
`|q − q_a| > drag_band_rad` the anchor is moved to stay `drag_band_rad` behind
`q`, so the spring contributes at most `K_s·drag_band_rad` (about 1 N·m on
joints 1-4, 0.3 N·m on 5-7). `required_modes()` is all torque; `command.position`
passes `q` through. The constructor throws on non-finite or negative gains, a
non-positive band, or a non-positive torque limit. The defaults are starting
values, untuned on the arm. See the
[guide](../guide/control-modes.md#float-floatmode).

### `CartesianImpedanceMode` — `cartesian_impedance_mode.h`

```cpp
struct CartesianImpedanceParams {
  Vector6 Kx = {300,300,300, 30,30,30};   // stiffness: N/m (xyz) | N·m/rad (rpy)
  Vector6 Dx = {35,35,35,  5,5,5};        // damping:   N·s/m     | N·m·s/rad
  double  nullspace_kp = 5.0;             // posture stiffness (N·m/rad)
  double  nullspace_kd = 1.0;             // posture damping  (N·m·s/rad)
  bool    nullspace_on = true;
  double  pinv_damping = 1e-3;            // λ for the damped pseudo-inverse
  double  torque_limit = 39.0;            // per-joint clamp (N·m)
  double  gain_ramp_s  = 0.5;             // active-wrench fade-in window (s)
};

CartesianImpedanceMode(Dynamics& dyn, CartesianImpedanceParams p = {});

// Non-RT setters — call from a single supervisor thread. Lock-free publish; the
// RT loop reads a consistent snapshot once per cycle. No allocation.
void set_target(const Pose& x_d) noexcept;
void set_gains(const CartesianImpedanceParams& p) noexcept;
```

Law: `tau = gravity + ramp·(Jᵀ(Kx∘e − Dx∘ẋ) + nullspace)`. Holds the entry pose
until `set_target` is called. `Vector6` gain layout is `[x y z | rx ry rz]`. See
the [guide](../guide/control-modes.md#cartesian-impedance-cartesianimpedancemode)
and the [Deep Dive](../deep-dive/impedance.md).

### `JointPositionMode` — `joint_position_mode.h`

```cpp
struct JointPositionParams {
  JointVec max_ref_speed = JointVec::Constant(0.5);   // rad/s, seeded/clamped to URDF
  double max_following_error = 0.35;                  // rad; <= 0 disables the leash
  JointVec q_lower = JointVec::Constant(-std::numeric_limits<double>::infinity());
  JointVec q_upper = JointVec::Constant( std::numeric_limits<double>::infinity());
                                                       // ^ seeded from URDF if non-finite
  double cmd_timeout_s = 0.0;                          // staleness watchdog; 0 disables
  double ik_fault_s    = 0.1;                          // sustained IK non-convergence, in TIME
  DiffIkParams ik{};
};
JointPositionMode(Dynamics& dyn, JointPositionParams p = {});

// Non-RT setters — call from a single supervisor thread. Lock-free publish; the
// RT loop reads a consistent snapshot once per cycle. No allocation.
void set_target(const JointVec& q_d) noexcept;   // JointTargetSink
void set_target(const Pose& x_d) noexcept;       // PoseTargetSink — resolved by in-loop IK
void set_command_timeout(double s) noexcept;

bool ik_faulted() const noexcept;   // latched once ik_fault_s of non-convergence elapses
IkResult last_ik() const noexcept;  // RT-owned, not synchronized
```

Runs no dynamics for a joint target — no gravity term, no mass matrix, no
IK — the cheapest control path in the driver. Implements both
`JointTargetSink` and `PoseTargetSink` (`joint_target_sink.h`,
`pose_target_sink.h`): a `Pose` target is resolved to a joint reference by the
same in-loop `DiffIkSolver` `JointImpedanceMode` uses, and the result feeds the
**same** `rate_limit → leash → wrap → clamp` reference pipeline a native joint
target does, so the pose path inherits the whole safety envelope unchanged.
`ik_faulted()` is published for the sampler thread to observe: the mode itself
freezes the reference immediately at 1 kHz on sustained non-convergence,
independent of anything upstream noticing, and `Supervisor`'s sampler loop then
closes any open streaming session with
`StreamCloseCause::kIkFault` (see
[the streaming guide](../guide/streaming.md#when-the-pose-path-cannot-solve)).
The latch is re-armed by `close_stream()` and on the transition into a pose
target, so a client that reconnects is not stuck behind the previous session's
fault. The IK seed is **persistent** across cycles — it is not re-seeded from
the rate-limited reference, or `converged` could never become true for a pose
more than one solve's travel away. See the
[guide](../guide/control-modes.md#joint-space-position-jointpositionmode).

### `JointVelocityMode` — `joint_velocity_mode.h`

```cpp
struct JointVelocityParams {
  JointVec max_qd = JointVec::Constant(std::numeric_limits<double>::infinity());
                                    // rad/s, seeded/clamped to URDF
  double dls_damping     = 1e-3;   // baseline LM damping for the twist solve
  double w_threshold      = 0.0033; // manipulability below which damping rises
  double dls_damping_max = 0.10;
  double posture_gain = 0.15;      // 1/s; null-space posture bias, 0 disables it
  JointVec q_rest = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  double cmd_timeout_s = 0.0;      // staleness watchdog; 0 disables
};
JointVelocityMode(Dynamics& dyn, JointVelocityParams p = {});

// Non-RT setters — call from a single supervisor thread. Lock-free publish; the
// RT loop reads a consistent snapshot once per cycle. Latest setter wins: a
// twist target supersedes a joint-velocity target and vice-versa.
void set_velocity_target(const JointVec& qd_d) noexcept;
void set_twist_target(const Vector6& V) noexcept;
void set_command_timeout(double s) noexcept;

JointVec commanded() const noexcept;          // RT-owned, not synchronized
double last_manipulability() const noexcept;  // sqrt(det(J Jᵀ)) at the last twist solve
```

Integrates the commanded velocity into a position reference at the RT rate and
commands every actuator in `kPosition`; the actuator's own position servo holds
the reference, so a zero command **holds** (the actuator's velocity servo does
not reject gravity at zero, [#34](https://github.com/rammp-org/kinova-gen3-driver/issues/34)).
The reference may lead the measured position by at most 0.1 rad per joint and
never leaves the URDF position limits. `out.velocity` is zero, as in every
position-commanding mode; `commanded()` is the limited velocity fed to the
integrator (when the leash bites, part of it is discarded).
**Stiff by contract** — this mode does not yield to contact and makes
no attempt to; a compliant velocity law is a different promise and belongs in a
different mode. Two target shapes: `set_velocity_target` is native
(pass-through, then uniformly-scaled-then-clamped to `max_qd`);
`set_twist_target` maps an EE twist `[linear; angular]` (base frame) by damped
least squares (`qd = Jᵀ(JJᵀ + λ²I)⁻¹V`) plus a projector-free null-space
posture bias toward `q_rest`. `λ` rises as manipulability
`w = sqrt(det(JJᵀ))` falls below `w_threshold`, which keeps the **solve**
well-conditioned near a singularity. What bounds the **command** is `limit()`:
a uniform scale so the fastest joint just reaches its cap, plus a hard per-joint
clamp, applied unconditionally. Since `w_threshold` sits at a tenth of the
nominal `w`, `limit()` is in practice the primary bound across the whole
near-singular band — reach for `dls_damping_max` when the solve is
ill-conditioned, not when the tool is merely sluggish. Staleness
commands **zero** velocity, freezes the reference at the measured position, and
**latches**, exactly like `JointImpedanceMode`'s freeze. See the [Deep Dive](../deep-dive/velocity-mode.md) for the full
derivation and the [guide](../guide/streaming.md#jointvelocitymode-specifics).

---

## `RtExecutor` — `rt_executor.h`

Owns the single RT thread, paces the loop, exchanges with the transport, runs the
active mode, and pushes a telemetry sample per cycle.

```cpp
enum class Pacing { kSleepSpin, kClockNanosleep };

struct ExecutorConfig {
  double   rate_hz = 1000.0;
  Pacing   pacing  = Pacing::kSleepSpin;
  RtConfig rt{};                 // see rt_system.h
  int      max_consecutive_misses = kMaxConsecutiveTimeouts;  // 3 exchange timeouts in a row => CommsLost
  int      max_window_misses = 5;       // ...or this many within miss_window => CommsLost
  int      miss_window = 1000;          // exchanges
  bool     exit_on_fault = false;       // a confirmed fault in feedback ends run() with ArmFault
};
class CommsLost : public std::runtime_error {};  // thrown by run()
class ArmFault : public std::runtime_error {};   // thrown by run() under exit_on_fault
inline constexpr int kFaultConfirmExchanges = 3;
inline constexpr int kLeftLowLevelConfirmExchanges = 50;

class RtExecutor {
 public:
  RtExecutor(Transport& t, SampleRing& ring, ExecutorConfig cfg);  // invalid_argument if a miss limit or miss_window < 1
  void request_mode(ControlMode* m) noexcept;   // adopted at the next cycle boundary
  void attach_contact(ContactMonitor& monitor, ControlMode& on_contact) noexcept;  // before run()
  void run(std::atomic<bool>& stop);            // blocks on the calling (RT) thread; may be called again
  const ExecutorConfig& config() const noexcept;
};
```

- `request_mode` stores a raw pointer adopted at a cycle boundary; the caller
  **retains ownership** and must keep the mode alive while active.
- `run` blocks until `stop` is set; it calls `enable_rt()` on its own thread.
- **Lost frames:** a `TransportTimeout` from `exchange` does not end the loop.
  That cycle keeps the last good feedback, skips `compute` (so the next cycle
  re-sends the previous command unchanged: no integration on stale feedback),
  defers any pending mode switch until a good exchange, and sets `CycleSample`
  bit2. Comms loss is `max_consecutive_misses` in a row, `max_window_misses`
  within the last `miss_window` exchanges (so a link losing every other frame
  can't run on forever), or a mode switch whose pumps time out: `run` throws
  `CommsLost` and the caller does `safe_shutdown()`. Any other exception from
  `exchange` (such as `ArmRejected`) propagates as before.
- **Faults (`exit_on_fault`):** off by default, so a caller that clears faults
  in place while the loop runs (the teleop app) keeps today's behaviour. On, a
  good exchange reporting `JointFeedback::fault` `kFaultConfirmExchanges` times
  in a row, or `!low_level` `kLeftLowLevelConfirmExchanges` times in a row,
  ends `run` with `ArmFault`. A miss neither confirms nor clears the count.
  `ArmLink` requires it.
- **Re-running:** `run` may be called again after it returns or throws; a mode
  requested in between is adopted by the new run (from a fresh seed read).
- **Pacing:** `kSleepSpin` (sleep-then-spin, lower jitter) vs `kClockNanosleep`
  (simpler). Both selectable for benchmarking.
- **Contact stop (`attach_contact`):** optional; call it before `run`, and keep
  `monitor` and `on_contact` (a `FloatMode`) alive as long as the executor. With
  none attached the loop is unchanged. Attached, `run` calls
  `monitor.on_run_start()` once at its start (the observer re-settles; the latch
  is kept) and `monitor.step(fb, dt, missed)` every cycle right after the
  exchange, a missed one included. The monitor shares the modes' `Dynamics`: it
  only runs on the RT thread. While `monitor.latched()`, `on_contact` is the only
  mode the loop runs:
  - **Trip cycle.** If `on_contact`'s actuator modes match the active mode's
    (every torque-controlled mode), the loop switches in that cycle and computes
    `on_contact`. Otherwise it sends a freeze that cycle (the active mode's
    actuator modes, `position` = measured `q`, zero `velocity`, no `compute`) and
    adopts `on_contact` at the next boundary through `set_actuator_modes`.
  - **Requests.** A `request_mode` made while latched, or racing the trip, is left
    pending, not dropped: the latest one is adopted at the first boundary after
    the clear (`ContactMonitor::request_clear`, consumed by the next `step`).
  - **Re-running.** A `run` that starts latched (after comms loss or an arm
    fault) adopts `on_contact` at its first boundary, not the pending request.
  - Each latched cycle sets `CycleSample` bit3. In `observe` the monitor never
    latches, so the commands are never changed.

---

## `ArmLink` — `arm_link.h`

Owns the link lifecycle so no arm-side event ends the process. Runs on the
thread that would otherwise call `RtExecutor::run()` and calls it while
`kRunning`; it owns no thread of its own.

```cpp
enum class LinkState { kConnecting, kRunning, kFaulted };
const char* to_string(LinkState);
struct LinkStatus { LinkState state; std::string reason; uint64_t faults, reconnects, clears_done; };
class LinkListener { public: virtual void on_link_state(LinkState) = 0; };
struct ArmLinkConfig {
  double retry_period_s = 1.0;   // between connect attempts, and after a link loss
  double fault_poll_hz = 100.0;  // receive() while faulted, so feedback stays live
  double health_period_s = 0.5;  // arm_in_fault() while faulted
  int health_max_failures = 3;   // unanswered in a row = link lost
  double clear_settle_s = 2.0;   // after a clear, how long the arm may take to leave IN_FAULT
};
class ArmLink {
 public:
  ArmLink(Transport&, RtExecutor&, LinkListener*, ArmLinkConfig = {});  // invalid_argument unless exit_on_fault
  void run(std::atomic<bool>& stop);  // returns on stop, transport shut down; only std::logic_error escapes
  bool request_clear();               // any thread; true only while kFaulted
  LinkStatus status() const;          // any thread
};
```

| Event | What `ArmLink` does | Next state |
|---|---|---|
| `connect()` fails (arm off or still booting, link renegotiating, no direct route to the arm, RPC timeout) | `safe_shutdown()`, retry every `retry_period_s`, forever | `kConnecting` |
| connected, base reports `ARMSTATE_IN_FAULT` | hand the arm back to the base (`release_low_level`) | `kFaulted` |
| connected, no fault | `set_servoing_low_level()`, every actuator to POSITION, `exec.run()` | `kRunning` |
| `CommsLost` or `TransportTimeout` out of the loop | latch `"connection lost: …"`, `safe_shutdown()`, reconnect | `kConnecting` → `kFaulted` |
| `TransportTimeout` before the loop ran (entering low-level) | `safe_shutdown()`, reconnect; no latch | `kConnecting` |
| `ArmFault`, `ArmRejected`, any other exception | latch the fault, `release_low_level()` | `kFaulted` |
| `request_clear()` | `clear_faults()`; still in fault after `clear_settle_s` → stays, reason `"fault persists after clear"` | `kRunning` |
| arm stops answering while faulted (`health_max_failures`) | `safe_shutdown()`, reconnect; **the fault stays latched** | `kConnecting` → `kFaulted` |
| `std::logic_error` from `connect()` (actuator-count mismatch, an arm address that isn't IPv4) | `safe_shutdown()`, rethrow | — |

`ArmLink` never clears a fault on its own initiative. The listener (the
`Supervisor`) is told every state change before it takes effect on the arm; see
`Supervisor::on_link_state`.

---

## Real-time system — `rt_system.h`

```cpp
struct RtConfig {
  int  priority = 80;            // SCHED_FIFO priority
  int  cpu = -1;                 // CPU affinity (-1 = no pinning)
  bool lock_memory = true;       // mlockall
  bool pin_cpu_latency = true;   // pin /dev/cpu_dma_latency to 0 µs (suppress deep C-states)
};
struct RtReport { bool mlock_ok; bool cpu_latency_pinned; int policy, priority, cpu; std::string note; };

RtReport enable_rt(const RtConfig&);   // best-effort; NEVER throws (degrades to SCHED_OTHER)
// SCHED_FIFO `priority` on another, already-running thread, then pin it to `cpu` (-1 = no pinning)
// only if the priority took. NEVER throws; returns "" on success, else what failed.
std::string raise_thread(pthread_t thread, int priority, int cpu);

struct ResourceUsage { uint64_t minflt, majflt, nvcsw, nivcsw; };
ResourceUsage read_usage();            // getrusage(RUSAGE_THREAD) — per calling thread
std::string introspect();              // applied sched policy/priority/affinity + usage, human-readable
```

`enable_rt` is best-effort: without privileges it logs a note and runs at
`SCHED_OTHER` (no hard-RT guarantees, but it still runs). `raise_thread` is for
threads the driver doesn't create, such as the KORTEX SDK's UDP receive thread
(`KortexTransport`'s `rx_priority`/`rx_cpu`). The driver needs **no
runtime sudo** — see [Real-Time Tuning](../rt-tuning.md). `majflt+=0` across a run
proves memory stayed locked.

---

## Transport — `transport.h`

The communication boundary; the only unit that includes KORTEX.

```cpp
class Transport {
 public:
  virtual void connect() = 0;
  virtual void set_servoing_low_level() = 0;
  virtual void set_actuator_modes(const ActuatorModes&) = 0;
  virtual void exchange(const JointCommand&, JointFeedback&) = 0;  // blocking round-trip
  virtual void send(const JointCommand&) = 0;                      // non-blocking
  virtual void receive(JointFeedback&) = 0;
  virtual void safe_shutdown() = 0;            // release_low_level() + close the connection
  virtual void release_low_level() = 0;        // actuators POSITION + base single-level; keeps the connection
  virtual bool arm_in_fault() = 0;             // base state over the control channel; TransportTimeout if unanswered
  virtual void clear_faults() {}               // re-enters low-level only if still in it
};
// a cyclic round-trip did not complete within the transport's cyclic timeout
class TransportTimeout : public std::runtime_error {};
// the arm refused a request (an error reply, e.g. WRONG_SERVOING_MODE)
class ArmRejected : public std::runtime_error {};
// TransportTimeouts in a row taken as the link being down; the default for
// ExecutorConfig::max_consecutive_misses and the limit for mode-switch pumps
inline constexpr int kMaxConsecutiveTimeouts = 3;
```

- **`SimTransport(const JointFeedback& initial, int latency_us = 0)`**
  (`sim_transport.h`) — fake robot for tests/benchmarks. Echoes state, advances a
  frame counter, optionally busy-waits `latency_us` to mimic the round-trip. No
  hardware. Test knobs for the link lifecycle: `fail_connects`, `set_arm_fault`,
  `set_fault_sticky`, `set_low_level_feedback`, `throw_on_exchange`,
  `fail_health`, `set_joint_positions`, and call counters.
  `set_external_torque(tau_ext)` simulates a push: feedback `tau` reports what it
  otherwise would minus `tau_ext`, so the contact observer reads `+tau_ext` (the
  arm does not move). Lock-free; safe to call while the loop runs.
  `set_gravity_model(g)` (before running): under a non-torque command, `tau`
  reports `g(q)`, the torque holding the arm at rest, instead of the last
  commanded torque (RtExecutor's seed sends zero torque, which a contact monitor
  would read as an external `g(q)`). `g` must not allocate; give it its own
  `Dynamics`. A kTorque command still echoes the commanded torque.
- **`KortexTransport(ip, user = "admin", password = "admin", rx_priority = 0, rx_cpu = -1, cyclic_timeout_ms = 20)`**
  (`kortex_transport.h`, KORTEX build only) — the real Gen3 low-level handshake.
  Compiled in only with `-DKINOVA_ENABLE_KORTEX=ON`. `connect()` first checks the
  route to the arm (`arm_route_problem`, `arm_route.h`): the arm's address must be on
  the network of the interface the kernel would send through, and that interface
  must have a link. If not, it throws `std::runtime_error` naming the interface at
  once, rather than letting KORTEX's TCP connect leave by the default route and block
  ~127 s; an address that isn't IPv4 is `std::invalid_argument`. It then moves the SDK's
  UDP receive thread — the one every `exchange()` waits on — to `SCHED_FIFO`
  `rx_priority` pinned to `rx_cpu` (via `raise_thread`, see
  [rt_system](#real-time-system-rt_systemh)); pass one below the executor's priority and its CPU.
  `rx_priority = 0` leaves it a normal thread. Best effort: the outcome goes to
  stderr either way. Every cyclic call on the RT
  path (`exchange`, `send`, `receive`, the pumps in `set_actuator_modes`) is
  bounded at `cyclic_timeout_ms` (KORTEX's default is 3000 ms); expiry is a
  `TransportTimeout`. The calls go through `IRouterClient::send` and wait on the
  reply with a `steady_clock` deadline, so a timeout is typed rather than
  recognised from KORTEX's error message. `set_actuator_modes` skips isolated lost
  pumps (`kMaxConsecutiveTimeouts` in a row rethrow). `set_servoing_low_level`'s
  seed reads keep 3000 ms. KORTEX errors are classified at this boundary: an
  error reply from the arm (`KDetailedException`) is `ArmRejected`; a failed
  local send (`sendto`) is a lost frame, `TransportTimeout`; a TCP RPC timeout
  (KORTEX's `"timeout detected"` `runtime_error`) is `TransportTimeout`.
  After the actuator count, `connect()` reads each actuator's control mode: a booting
  arm answers on its base before its actuators register, and that refusal is a plain
  `std::runtime_error` (retried), not an `ArmRejected`.
  `connect()` does not clear faults; an actuator-count mismatch is
  `std::invalid_argument`. `release_low_level()` and `arm_in_fault()` bound each
  RPC at 500 ms; the revert moves past a refusal and stops at the first RPC that goes unanswered. `fill_feedback`
  folds the base's fault banks and `ARMSTATE_IN_FAULT` into `fault` and sets
  `low_level` from the base's `active_state`. `set_servoing_low_level()` clears
  the retained gripper command, so re-entry leaves the gripper uncommanded, as at
  first startup.

Two decorators wrap a `Transport` rather than implementing one from scratch —
same shape, forward everything, and hook the one call each cares about:

- **`GripperController(Transport& inner)`** (`gripper_controller.h`) — stamps a
  `GripperCommand` onto every outgoing frame. See [Gripper](#gripper-gripper_typesh-gripper_controllerh-interfaceportsh).
- **`FeedbackTap(Transport& inner, Seqlock<JointFeedback>& snap)`**
  (`feedback_tap.h`) — publishes the latest `JointFeedback` into a `Seqlock` on
  every feedback-producing call, so a non-RT thread can read robot state
  without RtExecutor or the driver core knowing a reader exists.

---

## Telemetry — `telemetry.h`, `telemetry_consumers.h`

```cpp
struct CycleSample {
  uint64_t cycle_index;
  uint32_t wake_jitter_ns, comm_ns, compute_ns, cycle_ns;  // compute_ns includes the contact monitor's step
  uint16_t flags;          // bit0 overrun, bit1 fault, bit2 comm timeout, bit3 contact latched
};

class SampleRing {                       // lock-free SPSC, drop-don't-block
 public:
  explicit SampleRing(size_t capacity_pow2);
  bool push(const CycleSample&) noexcept;  // false => dropped (full)
  bool pop(CycleSample&) noexcept;         // false => empty
  uint64_t dropped() const noexcept;
};

class NanoHistogram {                    // log2-bucket histogram
  void add(uint32_t ns) noexcept;
  uint64_t count() const; uint32_t min() const; uint32_t max() const; double mean() const;
  uint32_t percentile(double p) const;   // LOWER bound of the log2 bucket (coarse, slightly low)
  std::string dump() const;
};

class TelemetrySink {
  explicit TelemetrySink(const std::string& csv_path = "");  // empty => no CSV
  void consume(const CycleSample&);
  std::string console_line() const;      // cumulative-since-construction summary
  const NanoHistogram& cycle_hist() const;
  const NanoHistogram& compute_hist() const;
};
```

The RT loop only ever calls `SampleRing::push` (never blocks — drops and bumps a
counter if full). All formatting/CSV/histogram work happens on a non-RT drain
thread that calls `pop` + `consume`. **`percentile` is coarse** (log2 buckets) —
fine for characterizing a 1 kHz loop, not an exact percentile; use the CSV for
precise analysis.

## Arbitration — `interface/arbiter.h`, `interface/ports.h`

`Arbiter` decides **who may command the arm**. It implements both `CommandSink`
and `StreamSink` and decorates a downstream instance of each (in practice the
same `Supervisor`), so it gates both the command and streaming tiers without
any coupling to control code. See the
[arbitration guide](../guide/arbitration.md) for the model.

```cpp
using Token = std::array<uint8_t, 16>;   // 128-bit capability token

enum class ArbitrationMode { kEnforced, kDisabled };
enum class HaltReason      { kOwnershipRevoked, kEmergencyStop, kOperatorRequest,
                             kArmUnavailable, kContact };  // the last two: raised by the Supervisor

Arbiter(CommandSink& downstream, StreamSink& downstream_stream, ArbitrationMode mode,
        uint64_t seed = 0);
```

`seed == 0` seeds the token RNG from `std::random_device`; a non-zero seed makes
tests deterministic.

### `ArbitrationSink` (driving port)

The orchestrator's side of the interface. The backend calls these; the `Arbiter`
implements them.

| Method | Effect |
|---|---|
| `GrantResult grant(const std::string& owner_id, const std::optional<Token>& token = std::nullopt)` | mints and returns a fresh token, or adopts `token` if given (an all-zero one is refused); halts first if the arm was already owned; refused while e-stopped |
| `void revoke()` | drops the grant and halts (`kOwnershipRevoked`) |
| `void estop()` | drops all grants, halts (`kEmergencyStop`), and **latches** |
| `void estop_clear()` | leaves the latch, to *no owner*; works in any mode |
| `ArbitrationStatus status() const` | mode, e-stop latch, owner, generation, rejection count |

### Gains contract — `interface/gains.h`, `interface/value_types.h`

```cpp
enum class GainsProfile { kSessionDefault, kSoft, kMedium, kStiff, kCustom };
struct ImpedanceGains {
  GainsProfile profile = GainsProfile::kSessionDefault;
  JointImpedanceGainValues custom{};  // read iff profile == kCustom
};
kinova::JointImpedanceParams profile_params(GainsProfile);  // named entries only; throws otherwise
GainsCheck validate_custom(const JointImpedanceGainValues&);      // {ok, message}; pure
bool known_profile(GainsProfile);  // accept-time guard: out-of-enum bytes die at the boundary
// base with ONLY the gain fields (Kq, zeta, leash, torque_limit) taken from profile
kinova::JointImpedanceParams overlay_profile_gains(const JointImpedanceParams& base, const JointImpedanceParams& profile);
kinova::JointImpedanceParams resolve_gains(const ImpedanceGains&, const JointImpedanceParams& session_default);
```

`TrajectoryGoal::gains` and `StreamOpenRequest::gains` carry an `ImpedanceGains`
(trajectory gains apply at execution, stream gains at open);
`GainsRequest::spec` carries one to `on_set_gains`, which validates and
replaces the **session default** — what an absent spec resolves to, initially
`kMedium` — and never touches a live session. Validation bounds are public
(`kKqMin/Max`, `kZetaMin/Max`, `kTorqueLimitFloor/Ceil`); the floors are the
per-joint gravity headroom, pinned from the measured worst-case URDF gravity
and re-derived by `GainsFloor.FloorsCoverWorstCaseGravityWithMargin`. Gains on
a surface where they cannot act (position goal, non-impedance stream) are
refused. A `JointImpedanceMode` exposes `params()` (RT-safe snapshot) for gain
read-back.

### `JointTarget` — `joint_target_sink.h`

```cpp
struct JointTarget { JointVec q, qd, qdd; bool has_velocity; bool has_acceleration; };
class JointTargetSink {
  virtual void set_joint_target(const JointTarget&) noexcept = 0;  // modes implement this
  void set_target(const JointVec& q_d) noexcept;  // convenience: JointTarget{q}, non-virtual
};
```

`interface::sample_target(tr, t_s)` returns the reference **and** the
derivatives of the very polynomial `sample()` evaluates; the executor scales
them into wall time under the speed scale (`qd·s`, `qdd·s²`) before handing
them to the sink. `JointImpedanceMode` consumes the profile as feedforward
(velocity-error damping + `M·qdd`); `JointPositionMode` ignores the
derivatives by design.

### `TrajectoryGoal::speed_scale` — `interface/value_types.h`

`TrajectoryGoal` carries `double speed_scale = 1.0`, in `[kMinSpeedScale,
1.0]` (`kMinSpeedScale = 0.01`): execute this goal's path slower by dilating
`TrajectoryExecutor`'s clock rather than altering the path itself — see the
[speed scale deep-dive](../deep-dive/trajectory-interpolation.md#speed-scale).
The floor exists because a scale of zero would stop the trajectory clock and
hang the goal forever; `effective_scale()` clamps to it internally, but that
clamp is unreachable from outside because two layers refuse anything below it.
A value outside `[kMinSpeedScale, 1.0]`, including non-finite, is **refused**
at `on_trajectory_goal` (`GoalResponse::kReject`) and again by
`TrajectoryExecutor::submit` (`SubmitResult::kRejectedSpeedScale`, settled as
`kInvalidGoal`) — the second layer covers a goal delivered straight to
`on_trajectory_accepted`, which never passes the first. Never clamped:
accepting a below-floor request and silently running it at the floor would be
*faster* than asked for, not slower.

### `CommandSink::on_set_speed_override(const SpeedOverrideRequest&)` / `Supervisor::set_speed_override(double)`

A runtime override an operator can use to slow every in-flight goal down,
independent of what the goal itself requested. `SpeedOverrideRequest{scale,
sender_id, token}` mirrors `GainsRequest`'s shape. Returns
`SpeedResult{accepted, message}`; a scale outside `[kMinSpeedScale, 1.0]`
(including non-finite) is refused and leaves the current override unchanged,
for the same floor reason as `speed_scale` above. The effective scale applied
by the executor is `min(goal.speed_scale, override)`, clamped to
`[kMinSpeedScale, 1.0]` and slew-limited at `kScaleSlewPerSec` — see the
[speed scale deep-dive](../deep-dive/trajectory-interpolation.md#speed-scale).

`on_set_speed_override` is gated **asymmetrically**, unlike every other
`CommandSink` method, and the two decisions live in two different places.
`Arbiter::on_set_speed_override` decides **authorisation only**: it sets
`SpeedOverrideRequest::may_raise = admit(token)` (which already checks both
the live token and the e-stop latch) and forwards the request
**unconditionally** — it never refuses the call itself. `may_raise` is
always overwritten on the way in; a client setting it has no effect.
`Supervisor::set_speed_override` decides **direction**: it refuses a request
where `may_raise` is false and `scale` is not strictly less than the
*current* `speed_override_`, comparing and storing as **one atomic
compare-and-store** against that value — not a separate load, compare, and
store, which would leave a window for a concurrent write to land in and make
the comparison stale. (An earlier version of this feature decided direction
in the `Arbiter`, against a copy of `ArmState::speed_override` — a snapshot
the pump loop refreshes only at `pump_hz`, up to one pump period stale. That
let an unauthenticated caller's request race a real lower and land as a
raise; deciding direction where the value is authoritative, atomically with
the store, closes the window.) *Lowering* is unaffected by `may_raise`: it
can only ever slow the arm down further, so it can never make the arm do
something it was not already doing, and gating it would let one client's
ownership block another operator's hand on the speed dial. *Raising* without
`may_raise` is refused: it can undo another operator's deliberate slow-down
of a moving arm, so an unauthenticated caller must not be able to do it.

### `CommandSink::on_halt(HaltReason)`

The general "stop the arm now" primitive — used by ownership revocation and
`/estop` alike. The caller declares **why**; the `Supervisor` decides **how**.
Every reason cancels and holds; every reason but `kContact` settles with
`result_code::kHalted` (−9), and `kContact` with `result_code::kContactStop`
(−11) — see [Contact stop](#contact-stop-haltreasonkcontact) below.

`on_halt` **must be idempotent**: `estop()` delivers it twice — once immediately,
before it contends for the arbiter's lock (so the arm stops without waiting on a
mode switch), and once while holding that lock, which is what orders the queue
flush after any command that had already been admitted. `Supervisor::on_halt`
satisfies this by construction: it latches a flag and clears a deque.

`Supervisor::on_halt` latches on the caller's thread and clears the queue; the
sampler thread performs the control action, which keeps `settle()`
single-threaded and therefore exactly-once. It settles the active goal **and**
every queued goal with `result_code::kHalted` (`result_code::kContactStop`, −11,
for `kContact`), then holds the active mode's target at the last-good
**measured** q. A goal the sampler had already taken from the queue when the
halt landed (for example while it slept through a mode settle) is settled the
same way instead of being started.

`HaltReason::kArmUnavailable` ("halted: arm fault or connection lost") is
raised by the `Supervisor` itself, never by a caller, from
`Supervisor::on_link_state` (it is `ArmLink`'s `LinkListener`). Leaving
`kRunning`, it refuses new goals and stream opens (`"arm not ready"`), halts,
then requests position mode for the next executor run (seeded from fresh
feedback, every old target dropped, so the arm holds where it is *now* after the
outage). Gripper setpoints are dropped while the arm is down, and a goal ACCEPTed
after the drop is settled `kHalted` rather than run. Entering `kRunning`
re-admits work. A `Supervisor` never told stays ready.

A halt (any reason) settles goals still waiting in the inbox with `kHalted`
(`kContactStop` for `kContact`) rather than dropping them. The pump publishes `ArmState` only when the feedback
snapshot is new since its last publish (`Seqlock::version()`), so a frozen
snapshot (link reconnecting, faulted polls unanswered, nothing read yet) is
never republished as live.

### Contact stop: `HaltReason::kContact`

With a `ContactMonitor` in `SupervisorDeps::contact` (attached to the executor
with `RtExecutor::attach_contact`), the RT loop latches a contact stop and
floats the arm itself; the `Supervisor` reacts at sampler rate.

- **Halt.** The sampler calls `on_halt(HaltReason::kContact)` once per rising
  edge of `ContactMonitor::latched()`. A clear resets the edge, so a re-trip
  after a clear halts again even if the sampler never saw the latch go down. The active and queued goals settle
  `result_code::kContactStop = -11` with `"halted: unexpected contact"`. A
  second halt landing before the sampler takes a pending contact halt (an
  ArmLink drop, an e-stop) keeps the contact reason.
- **While latched.** `on_trajectory_goal` rejects; `on_trajectory_accepted`
  settles −11; `on_stream_open` returns `{false, kContactStop, "stopped on
  unexpected contact; call /clear_faults"}`; gripper setpoints are dropped.
  `on_set_contact_thresholds` and `on_query_contact` still work. The latch
  survives everything but a clear: `on_link_state(kRunning)` does not unlatch,
  work stays refused, and the next executor run adopts float, not the position
  hold.
- **Clear.** `bool Supervisor::clear_contact()` (any thread; the node's
  `/clear_faults`) returns false and does nothing if nothing is latched.
  Otherwise it halts with `kContact` (a no-op repeat, unless the sampler has not
  seen this latch yet, for example during a 250 ms mode settle: then it is what
  settles a goal accepted before the trip), calls `request_clear()`, and
  requests position mode. The executor adopts only a pending request after a
  clear, and `JointPositionMode::on_enter` holds at the measured q. Pressing
  still when clearing re-trips after the observer settles (5/K).

Thresholds, from `interface/contact.h` and `interface/value_types.h`:

```cpp
enum class ContactProfile { kSensitive, kDefault, kLoose, kCustom };
struct ContactProfiles { JointVec sensitive, normal, loose, ceiling; };  // N·m; all NaN by default
ThresholdCheck validate_profiles(const ContactProfiles&);   // ceiling, then each named profile
ThresholdCheck resolve_contact(const ContactThresholdsRequest&, const ContactProfiles&,
                               JointVec& theta_out);        // unknown -> refused; then vs ceiling
struct ContactThresholdsRequest { ContactProfile profile = kDefault; JointVec custom; Token token; };
struct ContactThresholdsResult  { bool accepted; std::string message; ContactProfile profile; JointVec theta; };
struct ContactState { bool enabled; ContactProfile profile; ContactStatus status; };

virtual ContactThresholdsResult CommandSink::on_set_contact_thresholds(const ContactThresholdsRequest&);
virtual ContactState CommandSink::on_query_contact();
```

`on_set_contact_thresholds` refuses an unknown profile, resolves a named one
from `SupervisorDeps::contact_profiles` (`normal` is `kDefault`) or takes
`custom`, then refuses anything non-finite, ≤ 0 or above the ceiling, naming the
joint (1-based) and the limit. Nothing is clamped. A refusal leaves the active
thresholds as they were; accepted or not, `profile` and `theta` in the result
are what is active. An accepted set reaches the monitor within one RT cycle.
The `CommandSink` defaults (no monitor) are `{false, "contact detection not
available"}` and `ContactState{}` (`enabled = false`). `on_query_contact`
reports `enabled` (a monitor is attached), the last accepted `profile`
(`kDefault` at start: the constructor applies `contact_profiles.normal`), and
the monitor's `ContactStatus` (left default on a torn read).

### Gating

Every `CommandSink` method is gated except two outright, plus one that is
gated in only one of its two directions: `on_query_state()` (reads are always
open), `on_halt()` (a bare forward to the downstream `Supervisor` —
`Arbiter::on_halt` takes no lock and never calls `admit()`), and
`on_set_speed_override()`, which is ungated when *lowering* the effective
override and gated (token + e-stop latch, via `admit()`) when *raising* it —
see above. Halting is a safety action, one step stronger than the
lowering-a-speed-override reasoning: gating it would mean the arm could not be
stopped by whoever does not currently hold the token. `on_trajectory_accepted()`
re-checks the token on the goal rather than trusting that a matching
`on_trajectory_goal()` preceded it. `CancelRequest` exists so that cancel
carries a token too. `on_set_contact_thresholds()` is gated like `on_set_gains()`
(retuning the detector is a command); `on_query_contact()` is a read and is
not.

Rejections return `GoalResponse::kRejectUnauthorized` (goals),
`CancelResponse::kReject` (cancel), or `{accepted=false, "not authorized"}`
(gains, contact thresholds), and increment
`ArbitrationStatus::rejected_count`. `result_code::kNotAuthorized = -8` is the
constant for backends that produce a result message.

## Streaming — `interface/streaming_session.h`, `interface/ports.h`

The reactive counterpart to `CommandSink`'s trajectory goals: a driving port
for a client that publishes a setpoint every cycle instead of a plan to
interpolate. See the [streaming guide](../guide/streaming.md) for the
lifecycle model and the valid-pair table.

```cpp
enum class SetpointKind { kJointPosition, kEePose, kJointVelocity, kEeTwist, kJointTorque };

struct StreamOpenRequest {
  SetpointKind    kind         = SetpointKind::kJointPosition;
  ControlModeKind control_mode = ControlModeKind::kPosition;
  double          timeout_s    = 0.1;   // <= 0 is REJECTED at open: no deadline, no safe-stop
  Token           token{};
};
struct StreamOpenResult   { bool accepted=false; int error_code=0; std::string message; };
struct StreamCloseRequest { Token token{}; };

// One struct, three meanings -- units are per-method: rad (position), rad/s
// (velocity), N*m (feedforward torque).
struct JointSetpoint { JointVec values = JointVec::Zero(); Token token{}; };
struct PoseSetpoint  { Pose     pose{};                    Token token{}; };
struct TwistSetpoint { Vector6  twist = Vector6::Zero();   Token token{}; };  // [linear; angular], base frame
```

Each setpoint struct is an **absolute** target, never a delta — resending one
is idempotent, and the single-writer double-buffer under it means an
intermediate setpoint that arrives between two RT cycles is correctly dropped
rather than lost.

### `StreamSink` (driving port)

```cpp
class StreamSink {
 public:
  virtual StreamOpenResult on_stream_open(const StreamOpenRequest&) = 0;
  virtual void             on_stream_close(const StreamCloseRequest&) = 0;
  virtual void             on_setpoint_joint_position(const JointSetpoint&) = 0;
  virtual void             on_setpoint_joint_velocity(const JointSetpoint&) = 0;
  virtual void             on_setpoint_joint_torque(const JointSetpoint&) = 0;
  virtual void             on_setpoint_pose(const PoseSetpoint&) = 0;
  virtual void             on_setpoint_twist(const TwistSetpoint&) = 0;
};
```

`Supervisor` implements this (and `Arbiter` decorates it, gating on the same
token as `CommandSink` — see [Arbitration](#arbitration-interfacearbiterh-interfaceportsh)).
Only one session may be open at a time, and `on_stream_open` is refused while a
trajectory goal is in flight, and vice versa for `on_trajectory_goal` — the two
tiers never write the active mode's target concurrently.

### `bool pair_supported(SetpointKind, ControlModeKind)`

The single source of truth for which (setpoint kind, control mode) pairs this
driver can execute, checked at `on_stream_open` so an unsupported pair is
refused loudly rather than silently mis-driven:

| Setpoint kind | Control mode | Supported? |
|---|---|---|
| `kJointPosition` | `kPosition` / `kImpedance` | yes |
| `kEePose` | `kImpedance` | yes — resolved by `JointImpedanceMode`'s in-loop IK |
| `kEePose` | `kPosition` | yes — resolved by `JointPositionMode`'s in-loop IK |
| `kJointTorque` | `kTorque` | yes |
| `kJointVelocity` | `kVelocity` | yes — native `JointVelocityMode::set_velocity_target` |
| `kEeTwist` | `kVelocity` | yes — resolved by `JointVelocityMode`'s damped-least-squares twist map |

Every other `(kind, control mode)` combination is refused loudly at
`on_stream_open` rather than silently mis-driven — this table is the single
source of truth for exactly which combinations exist.

`on_stream_open` also refuses `timeout_s <= 0` unconditionally: the deadline
that lets the driver detect and tear down a dead stream is not optional, since
an unbounded stream has no safe-stop if the client disappears mid-session.

### Why the last session ended — `StreamCloseCause`

```cpp
enum class StreamCloseCause { kNone, kClientRequest, kDeadlineExpired, kHalted, kIkFault };
StreamCloseCause Supervisor::stream_close_cause() const;   // the LAST close
```

Every path through `Supervisor::close_stream()` records one of these. The
distinction that matters operationally is `kDeadlineExpired` versus `kIkFault`:
the first says the client went quiet and re-opening is the right response, the
second says the driver could not solve for the poses being streamed, and
re-opening the same session will just reproduce it. See
[the streaming guide](../guide/streaming.md#when-the-pose-path-cannot-solve).

### `CommandWatchdog` — `command_watchdog.h`

Shared staleness **detection**, used by all four of `JointTorqueMode`,
`JointPositionMode`, `JointImpedanceMode`, and `JointVelocityMode`. It is a lock-free counter bumped
by every non-RT setter and ticked once per RT cycle against an armed timeout —
no clock call, no allocation, so it is safe on the RT path. What a mode *does*
about staleness is deliberately not this class's job — each mode owns that
response (torque ramps `tau_ff` to zero via `cmd_decay_s`; position and
impedance freeze the reference at measured q). `JointTorqueParams`,
`JointPositionParams`, and `JointImpedanceParams` each carry a `cmd_timeout_s`
field (`<= 0` disables it, which is every mode's default except torque's
`0.1`), and each mode exposes:

```cpp
// s >= 0 arms the watchdog with s; s < 0 restores this mode's own configured
// default (params().cmd_timeout_s / p_.cmd_timeout_s).
void set_command_timeout(double s) noexcept;
```

`Supervisor::on_stream_open` calls this with the session's `timeout_s` on
whichever mode the stream targets; `close_stream()` calls it with a negative
value to hand the mode back to its own configured default rather than
disabling its watchdog outright.

## Gripper — `gripper_types.h`, `gripper_controller.h`, `interface/ports.h`

The gripper is not a `ControlMode` — see the [guide](../guide/gripper.md) for
why. `GripperController` decorates `Transport`; `GripperSink` is the driving
port `Supervisor` implements and `Arbiter` decorates, gated on the same token
as `CommandSink` and `StreamSink`.

```cpp
// gripper_types.h — what core exchanges with the transport, normalized 0..1.
struct GripperCommand {
  float position = 0.0f;   // 0 (open) .. 1 (closed)
  float speed    = 1.0f;   // fraction of max closing speed
  float force    = 0.5f;   // fraction of max — a CEILING on motor current, not
                            // a force setpoint; no force servo exists on this
                            // hardware by any path
  bool  active   = false;  // false => no gripper command emitted at all
};

inline constexpr float kGripperMaxCurrentA = 1.0f;   // MEASURED peak grip current

struct GripperFeedback {
  float position = 0.0f;   // 0 (open) .. 1 (closed)
  float effort   = 0.0f;   // 0..1, |current| / kGripperMaxCurrentA — NOT Newtons
  float current  = 0.0f;   // amps, raw, exactly as reported
  bool  present  = false;  // false when no interconnect gripper is attached
  // Deliberately NO velocity: MotorFeedback's field was measured to be the
  // commanded speed echoed back, not a measurement — see the guide.
};
```

```cpp
// gripper_controller.h
class GripperController : public Transport {
 public:
  explicit GripperController(Transport& inner);
  // Non-RT, one writer. Latest wins; every call carries all three fields —
  // speed and force are NOT sticky (see the guide's statelessness note).
  // Clamps position/speed/force to [0, 1].
  void set_target(const GripperCommand& c) noexcept;
  // Non-RT, the halt path. Stops STAMPING (cmd.gripper.active = false next
  // cycle) — it does NOT stop commanding on KortexTransport, which
  // retransmits the last cyclic command whole every cycle regardless of
  // `active`. The grip holds because the 2F-85 self-locks, not because the
  // motor goes idle; see the guide's "On halt, the gripper holds".
  void release() noexcept;
  GripperCommand target() const noexcept;   // RT-owned view, NOT synchronized
  // ... Transport passthrough ...
};
```

Law: forwards every `Transport` call unchanged, and on `exchange`/`send`
stamps the last-published `GripperCommand` into the outgoing `JointCommand`.
No allocation, lock, or blocking call — the stamp is a fixed-size struct copy.

```cpp
// interface/value_types.h
struct GripperSetpoint { kinova::GripperCommand command{}; Token token{}; };
struct GripperState {
  float  position = 0.0f;
  float  effort   = 0.0f;
  float  current  = 0.0f;   // amps
  bool   present  = false;
  double stamp_s  = 0.0;
};
```

`GripperSetpoint` carries the command and a token, exactly like `JointSetpoint`
and the other streaming setpoints — every command carries its own authority,
and the token checked is the **arm's** (spec decision 1: one physical machine,
one holder, no separate gripper grant). `GripperState` mirrors
`GripperFeedback` plus a stamp.

```cpp
// interface/ports.h
class GripperSink {
 public:
  virtual ~GripperSink() = default;
  virtual void         on_gripper_setpoint(const GripperSetpoint&) = 0;
  virtual GripperState on_query_gripper() = 0;
};
```

Two methods, not four: the `Grasp` action and its cancel were cut with stall
detection, so this tier is topic-only — set a target, read state back. Like
`on_query_state()`, `on_query_gripper()` is never gated: reads are always open.
`Supervisor::on_halt` calls `GripperController::release()` and nothing else —
a deliberate no-op beyond that, so the gripper holds through a halt (see the
[guide](../guide/gripper.md#on-halt-the-gripper-holds)).

### `SupervisorDeps` — `interface/supervisor.h`

```cpp
struct SupervisorDeps {
  JointPositionMode*      pos      = nullptr;
  JointImpedanceMode*     imp      = nullptr;
  JointTorqueMode*        tau      = nullptr;
  JointVelocityMode*      vel      = nullptr;
  RtExecutor*             exec     = nullptr;
  Seqlock<JointFeedback>* snap     = nullptr;
  Dynamics*               pump_dyn = nullptr;
  StreamPort*             stream   = nullptr;
  ActionServerPort*       action   = nullptr;
  GripperController*      grip     = nullptr;   // OPTIONAL — null on a robot with no gripper
  ContactMonitor*         contact  = nullptr;   // OPTIONAL — null: contact detection off
  ContactProfiles         contact_profiles{};   // read iff contact is set
  SupervisorConfig        cfg{};
};
```

Named dependencies for the `Supervisor` constructor in place of positional
arguments — every field but `grip` is `require()`-checked and throws naming
the missing one at construction (fail loud at startup, not at 1 kHz). `grip`
is the one field that is legitimately absent: `GripperFeedback::present`
already exists for a robot with no interconnect gripper attached, so a null
`grip` makes gripper commands no-ops and `on_query_gripper()` report
`present = false` rather than being a construction error.

`contact` is optional too: null means no contact stop, with
`on_set_contact_thresholds` refused and `on_query_contact().enabled == false`.
When it is set, the constructor throws `std::invalid_argument` unless
`validate_profiles(contact_profiles)` passes (the message names the profile,
the joint and the limit) and `contact_profiles.ceiling` equals
`contact->ceiling()`. It then applies the `kDefault` profile
(`contact_profiles.normal`). Attach the same monitor to the executor.
