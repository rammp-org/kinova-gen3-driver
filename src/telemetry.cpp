#include "kinova_lowlevel/telemetry.h"

namespace kinova {

namespace {
// Round up to next power of two, minimum 2.
size_t round_up_pow2(size_t n) {
  if (n < 2) return 2;
  size_t p = 2;
  while (p < n) p <<= 1;
  return p;
}
}  // namespace

SampleRing::SampleRing(size_t capacity_pow2)
    : buf_(round_up_pow2(capacity_pow2)), mask_(buf_.size() - 1) {}

bool SampleRing::push(const CycleSample& s) noexcept {
  const uint64_t head = head_.load(std::memory_order_relaxed);
  const uint64_t tail = tail_.load(std::memory_order_acquire);
  if (head - tail >= buf_.size()) {
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  buf_[head & mask_] = s;
  head_.store(head + 1, std::memory_order_release);
  return true;
}

bool SampleRing::pop(CycleSample& out) noexcept {
  const uint64_t tail = tail_.load(std::memory_order_relaxed);
  const uint64_t head = head_.load(std::memory_order_acquire);
  if (tail == head) {
    return false;
  }
  out = buf_[tail & mask_];
  tail_.store(tail + 1, std::memory_order_release);
  return true;
}

uint64_t SampleRing::dropped() const noexcept {
  return dropped_.load(std::memory_order_relaxed);
}

}  // namespace kinova
