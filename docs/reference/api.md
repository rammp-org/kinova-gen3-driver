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

Commands every actuator in `kVelocity` and lets the actuator's own servo close
the loop. **Stiff by contract** — this mode does not yield to contact and makes
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
commands **zero** velocity and **latches**, exactly like `JointImpedanceMode`'s
freeze. See the [Deep Dive](../deep-dive/velocity-mode.md) for the full
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
};

class RtExecutor {
 public:
  RtExecutor(Transport& t, SampleRing& ring, ExecutorConfig cfg);
  void request_mode(ControlMode* m) noexcept;   // adopted at the next cycle boundary
  void run(std::atomic<bool>& stop);            // blocks on the calling (RT) thread
};
```

- `request_mode` stores a raw pointer adopted at a cycle boundary; the caller
  **retains ownership** and must keep the mode alive while active.
- `run` blocks until `stop` is set; it calls `enable_rt()` on its own thread.
- **Pacing:** `kSleepSpin` (sleep-then-spin, lower jitter) vs `kClockNanosleep`
  (simpler). Both selectable for benchmarking.

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

struct ResourceUsage { uint64_t minflt, majflt, nvcsw, nivcsw; };
ResourceUsage read_usage();            // getrusage(RUSAGE_THREAD) — per calling thread
std::string introspect();              // applied sched policy/priority/affinity + usage, human-readable
```

`enable_rt` is best-effort: without privileges it logs a note and runs at
`SCHED_OTHER` (no hard-RT guarantees, but it still runs). The driver needs **no
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
  virtual void safe_shutdown() = 0;
};
```

- **`SimTransport(const JointFeedback& initial, int latency_us = 0)`**
  (`sim_transport.h`) — fake robot for tests/benchmarks. Echoes state, advances a
  frame counter, optionally busy-waits `latency_us` to mimic the round-trip. No
  hardware.
- **`KortexTransport(const std::string& ip)`** (`kortex_transport.h`, KORTEX build
  only) — the real Gen3 low-level handshake. Compiled in only with
  `-DKINOVA_ENABLE_KORTEX=ON`.

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
  uint32_t wake_jitter_ns, comm_ns, compute_ns, cycle_ns;
  uint16_t flags;          // bit0 overrun, bit1 fault
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
enum class HaltReason      { kOwnershipRevoked, kEmergencyStop, kOperatorRequest };

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
| `GrantResult grant(const std::string& owner_id)` | mints and returns a fresh token; halts first if the arm was already owned; refused while e-stopped |
| `void revoke()` | drops the grant and halts (`kOwnershipRevoked`) |
| `void estop()` | drops all grants, halts (`kEmergencyStop`), and **latches** |
| `void estop_clear()` | leaves the latch, to *no owner*; works in any mode |
| `ArbitrationStatus status() const` | mode, e-stop latch, owner, generation, rejection count |

### `CommandSink::on_halt(HaltReason)`

The general "stop the arm now" primitive — used by ownership revocation and
`/estop` alike. The caller declares **why**; the `Supervisor` decides **how**.
In v1 every reason produces the same action: cancel and hold.

`on_halt` **must be idempotent**: `estop()` delivers it twice — once immediately,
before it contends for the arbiter's lock (so the arm stops without waiting on a
mode switch), and once while holding that lock, which is what orders the queue
flush after any command that had already been admitted. `Supervisor::on_halt`
satisfies this by construction: it latches a flag and clears a deque.

`Supervisor::on_halt` latches on the caller's thread and clears the queue; the
sampler thread performs the control action, which keeps `settle()`
single-threaded and therefore exactly-once. It settles the active goal **and**
every queued goal with `result_code::kHalted`, then holds the active mode's
target at the last-good **measured** q.

### Gating

Every `CommandSink` method is gated except `on_query_state()` — reads are always
open. `on_trajectory_accepted()` re-checks the token on the goal rather than
trusting that a matching `on_trajectory_goal()` preceded it. `CancelRequest`
exists so that cancel carries a token too.

Rejections return `GoalResponse::kRejectUnauthorized` (goals),
`CancelResponse::kReject` (cancel), or `{accepted=false}` (gains), and increment
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
