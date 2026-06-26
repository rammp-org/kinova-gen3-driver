#pragma once
// Wire protocol for the teleop socket server. This is the C++ half of the
// contract defined in the Python supervisor's `kinova_teleop/protocol.py`; the
// two MUST stay byte-for-byte identical. Layout is little-endian, packed (no
// padding). Quaternions are serialized w, x, y, z (Eigen Quaterniond order).
// Positions are meters in the robot base frame. Bump kVersion on any change and
// update the Python side; the static_asserts below guard the C++ sizes.
//
// Endianness note: x86_64 and aarch64 (Jetson) are little-endian, matching the
// Python `<` struct format. This header assumes a little-endian host.
#include <cstdint>

namespace kinova::teleop {

inline constexpr uint32_t kMagic = 0x4B544C50;  // "KTLP"
inline constexpr uint16_t kVersion = 1;
inline constexpr int kNumJointsProto = 7;

enum class MsgType : uint16_t {
  kPoseTarget = 1,
  kSetGains = 2,
  kControl = 3,
  kFeedback = 4,
};

enum class ControlCmd : uint32_t {
  kClearFaults = 1,
  kRehome = 2,
  kFreeze = 3,
  kShutdown = 4,
};

enum TargetFlags : uint32_t {
  kFlagNone = 0,
  kFlagEngaged = 1u << 0,
  kFlagFreeze = 1u << 1,
};

#pragma pack(push, 1)

struct Header {
  uint32_t magic;
  uint16_t version;
  uint16_t msg_type;
  uint32_t seq;
  uint64_t timestamp_ns;
};
static_assert(sizeof(Header) == 20, "Header layout must match Python");

struct PoseTargetPacket {
  Header h;
  double pos[3];          // x, y, z [m], base frame
  double quat_wxyz[4];    // w, x, y, z
  float gripper;          // 0=open .. 1=closed
  uint32_t flags;         // TargetFlags
};
static_assert(sizeof(PoseTargetPacket) == 84, "PoseTarget layout mismatch");

struct GainsPacket {
  Header h;
  double Kx[6];           // [x y z | rx ry rz]
  double Dx[6];
  double nullspace_kp;
  double nullspace_kd;
  double pinv_damping;
  double torque_limit;
  double gain_ramp_s;
  uint8_t nullspace_on;
};
static_assert(sizeof(GainsPacket) == 157, "Gains layout mismatch");

struct ControlPacket {
  Header h;
  uint32_t command;       // ControlCmd
  uint32_t control_seq;
};
static_assert(sizeof(ControlPacket) == 28, "Control layout mismatch");

struct FeedbackPacket {
  Header h;
  double q[7];
  double qd[7];
  double tau[7];
  double ee_pos[3];
  double ee_quat_wxyz[4];
  float gripper_state;
  uint8_t fault;
  uint64_t frame_id;
  uint32_t last_control_seq;
};
static_assert(sizeof(FeedbackPacket) == 261, "Feedback layout mismatch");

#pragma pack(pop)

inline bool valid_header(const Header& h, MsgType expected, uint32_t /*size_seen*/) {
  return h.magic == kMagic && h.version == kVersion &&
         h.msg_type == static_cast<uint16_t>(expected);
}

}  // namespace kinova::teleop
