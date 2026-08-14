#pragma once
// FeedbackTap + Seqlock — capture the arm's latest JointFeedback for a non-RT
// reader without touching the driver core. A FeedbackTap decorates the Transport
// and snapshots `fb` at the exact point the RT loop reads it; a non-RT thread
// (teleop feedback stream, trajectory divergence guard) reads the latest snapshot
// through the lock-free Seqlock. RT-safe on the write side: no alloc, lock, or
// blocking call.
#include <atomic>
#include <cstdint>

#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/transport.h"

namespace kinova {

// Single-writer / multi-reader lock-free latest-value snapshot (seqlock). One
// thread (the RT thread) is the only writer; any number of non-RT threads may
// read concurrently (readers never mutate seq_/data_, so concurrent reads are safe).
// T need not be trivially copyable — fixed-size Eigen members store data inline,
// so a torn read yields transient garbage numbers but never a bad pointer.
template <class T>
class Seqlock {
 public:
  void store(const T& v) {  // writer (RT thread): bounded, no alloc/lock
    const uint32_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_release);  // mark write in progress (odd)
    std::atomic_thread_fence(std::memory_order_release);
    data_ = v;
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(s + 2, std::memory_order_release);  // publish (even)
  }
  bool load(T& out) const {  // reader: retry while a write is in flight
    for (int i = 0; i < 16; ++i) {
      const uint32_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 & 1u) continue;
      std::atomic_thread_fence(std::memory_order_acquire);
      out = data_;
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq_.load(std::memory_order_acquire) == s1) return true;
    }
    return false;
  }

 private:
  std::atomic<uint32_t> seq_{0};
  T data_{};
};

// Transport decorator: forwards everything to the wrapped transport and, on each
// feedback-producing call, publishes the latest JointFeedback into a snapshot.
// This is what lets a non-RT thread read robot state without any change to
// RtExecutor or the driver library.
class FeedbackTap : public Transport {
 public:
  FeedbackTap(Transport& inner, Seqlock<JointFeedback>& snap)
      : inner_(inner), snap_(snap) {}
  void connect() override { inner_.connect(); }
  void set_servoing_low_level() override { inner_.set_servoing_low_level(); }
  void set_actuator_modes(const ActuatorModes& m) override {
    inner_.set_actuator_modes(m);
  }
  void exchange(const JointCommand& c, JointFeedback& fb) override {
    inner_.exchange(c, fb);
    snap_.store(fb);
  }
  void send(const JointCommand& c) override { inner_.send(c); }
  void receive(JointFeedback& fb) override {
    inner_.receive(fb);
    snap_.store(fb);
  }
  void safe_shutdown() override { inner_.safe_shutdown(); }
  void clear_faults() override { inner_.clear_faults(); }

 private:
  Transport& inner_;
  Seqlock<JointFeedback>& snap_;
};

}  // namespace kinova
