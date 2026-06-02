#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include "kinova_lowlevel/telemetry.h"
namespace kinova {
class NanoHistogram {
 public:
  void add(uint32_t ns) noexcept;
  uint64_t count() const noexcept;
  uint32_t min() const noexcept;
  uint32_t max() const noexcept;
  double mean() const noexcept;
  uint32_t percentile(double p) const noexcept;   // p in [0,1]; returns bucket lower bound
  std::string dump() const;                        // CSV-ish bucket table
 private:
  std::vector<uint64_t> buckets_ = std::vector<uint64_t>(64, 0);  // bucket k = [2^k, 2^(k+1))
  uint64_t count_ = 0;
  uint64_t sum_ = 0;
  uint32_t min_ = UINT32_MAX;
  uint32_t max_ = 0;
};
class TelemetrySink {
 public:
  explicit TelemetrySink(const std::string& csv_path = "");  // empty => no CSV
  ~TelemetrySink();
  void consume(const CycleSample&);
  std::string console_line() const;        // one-line rolling summary
  const NanoHistogram& cycle_hist() const { return cycle_; }
  const NanoHistogram& compute_hist() const { return compute_; }
 private:
  NanoHistogram cycle_, compute_, comm_, jitter_;
  uint64_t overruns_ = 0, faults_ = 0, n_ = 0;
  void* csv_ = nullptr;   // FILE*
};
}  // namespace kinova
