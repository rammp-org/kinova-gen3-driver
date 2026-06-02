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
  // p in [0,1]. Returns the LOWER BOUND of the log2 bucket the percentile falls
  // in, so reported values are coarse and slightly low-biased (e.g. 3000ns reads
  // as 2048). Fine for characterizing a 1 kHz loop; not an exact percentile.
  uint32_t percentile(double p) const noexcept;
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
  // One-line summary. NOTE: percentiles/max are CUMULATIVE since construction,
  // not since the last call — for interval/rolling analysis use the CSV.
  std::string console_line() const;
  const NanoHistogram& cycle_hist() const { return cycle_; }
  const NanoHistogram& compute_hist() const { return compute_; }
 private:
  NanoHistogram cycle_, compute_, comm_, jitter_;
  uint64_t overruns_ = 0, faults_ = 0, n_ = 0;
  void* csv_ = nullptr;   // FILE*
};
}  // namespace kinova
