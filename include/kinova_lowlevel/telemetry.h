#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>
namespace kinova {
struct CycleSample {
  uint64_t cycle_index = 0;
  uint32_t wake_jitter_ns = 0;
  uint32_t comm_ns = 0;
  uint32_t compute_ns = 0;
  uint32_t cycle_ns = 0;
  uint16_t flags = 0;          // bit0 overrun, bit1 fault
};
class SampleRing {
 public:
  explicit SampleRing(size_t capacity_pow2);
  bool push(const CycleSample&) noexcept;   // false => dropped (full)
  bool pop(CycleSample&) noexcept;          // false => empty
  uint64_t dropped() const noexcept;
 private:
  std::vector<CycleSample> buf_;
  size_t mask_ = 0;
  std::atomic<uint64_t> head_{0};
  std::atomic<uint64_t> tail_{0};
  std::atomic<uint64_t> dropped_{0};
};
}  // namespace kinova
