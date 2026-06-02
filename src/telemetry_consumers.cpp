#include "kinova_lowlevel/telemetry_consumers.h"

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace kinova {

// ---- NanoHistogram ----------------------------------------------------------

void NanoHistogram::add(uint32_t ns) noexcept {
  const int k = ns ? (63 - __builtin_clzll(static_cast<uint64_t>(ns))) : 0;
  const int idx = k > 63 ? 63 : k;
  ++buckets_[idx];
  ++count_;
  sum_ += ns;
  if (ns < min_) min_ = ns;
  if (ns > max_) max_ = ns;
}

uint64_t NanoHistogram::count() const noexcept { return count_; }

uint32_t NanoHistogram::min() const noexcept {
  return count_ ? min_ : 0;
}

uint32_t NanoHistogram::max() const noexcept { return max_; }

double NanoHistogram::mean() const noexcept {
  return count_ ? static_cast<double>(sum_) / static_cast<double>(count_) : 0.0;
}

uint32_t NanoHistogram::percentile(double p) const noexcept {
  if (count_ == 0) return 0;
  if (p < 0.0) p = 0.0;
  if (p > 1.0) p = 1.0;
  uint64_t target = static_cast<uint64_t>(std::ceil(p * static_cast<double>(count_)));
  if (target == 0) target = 1;
  uint64_t acc = 0;
  for (int k = 0; k < 64; ++k) {
    acc += buckets_[k];
    if (acc >= target) {
      // Return the bucket lower bound 2^k. Shift on uint32_t and clamp k<=31
      // so the shift can never be UB or overflow the uint32_t return type.
      return std::uint32_t(1) << (k > 31 ? 31 : k);
    }
  }
  return std::uint32_t(1) << 31;  // unreachable (all counts accounted for above)
}

std::string NanoHistogram::dump() const {
  std::string out = "lo,hi,count\n";
  char line[64];
  for (int k = 0; k < 64; ++k) {
    if (buckets_[k] == 0) continue;
    const uint64_t lo = 1ull << k;
    const uint64_t hi = 1ull << (k + 1);
    std::snprintf(line, sizeof(line), "%llu,%llu,%llu\n",
                  static_cast<unsigned long long>(lo),
                  static_cast<unsigned long long>(hi),
                  static_cast<unsigned long long>(buckets_[k]));
    out += line;
  }
  return out;
}

// ---- TelemetrySink ----------------------------------------------------------

TelemetrySink::TelemetrySink(const std::string& csv_path) {
  if (!csv_path.empty()) {
    FILE* f = std::fopen(csv_path.c_str(), "w");
    csv_ = f;
    if (f) {
      std::fprintf(f, "cycle_index,wake_jitter_ns,comm_ns,compute_ns,cycle_ns,flags\n");
    }
  }
}

TelemetrySink::~TelemetrySink() {
  if (csv_) {
    std::fclose(static_cast<FILE*>(csv_));
    csv_ = nullptr;
  }
}

void TelemetrySink::consume(const CycleSample& s) {
  cycle_.add(s.cycle_ns);
  compute_.add(s.compute_ns);
  comm_.add(s.comm_ns);
  jitter_.add(s.wake_jitter_ns);
  overruns_ += (s.flags & 1u) ? 1 : 0;
  faults_ += (s.flags & 2u) ? 1 : 0;
  ++n_;
  if (csv_) {
    std::fprintf(static_cast<FILE*>(csv_), "%llu,%u,%u,%u,%u,%u\n",
                 static_cast<unsigned long long>(s.cycle_index),
                 s.wake_jitter_ns, s.comm_ns, s.compute_ns, s.cycle_ns,
                 static_cast<unsigned>(s.flags));
  }
}

std::string TelemetrySink::console_line() const {
  char buf[384];
  std::snprintf(
      buf, sizeof(buf),
      "n=%llu cycle[p50=%.1f p99=%.1f p99.9=%.1f max=%.1f]us "
      "comm[p99=%.1f]us compute[p99=%.1f]us jitter[p99=%.1f]us "
      "overruns=%llu faults=%llu",
      static_cast<unsigned long long>(n_),
      cycle_.percentile(0.5) / 1000.0,
      cycle_.percentile(0.99) / 1000.0,
      cycle_.percentile(0.999) / 1000.0,
      cycle_.max() / 1000.0,
      comm_.percentile(0.99) / 1000.0,
      compute_.percentile(0.99) / 1000.0,
      jitter_.percentile(0.99) / 1000.0,
      static_cast<unsigned long long>(overruns_),
      static_cast<unsigned long long>(faults_));
  return std::string(buf);
}

}  // namespace kinova
