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
