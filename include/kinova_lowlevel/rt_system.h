#pragma once
#include <cstdint>
#include <string>
namespace kinova {
struct RtConfig { int priority = 80; int cpu = -1; bool lock_memory = true; bool pin_cpu_latency = true; };
struct RtReport { bool mlock_ok = false; bool cpu_latency_pinned = false; int policy = -1; int priority = -1; int cpu = -1; std::string note; };
RtReport enable_rt(const RtConfig&);            // best-effort; never throws
struct ResourceUsage { uint64_t minflt = 0, majflt = 0, nvcsw = 0, nivcsw = 0; };
ResourceUsage read_usage();                     // getrusage(RUSAGE_THREAD)
std::string introspect();                       // applied sched + affinity + usage, human-readable
}  // namespace kinova
