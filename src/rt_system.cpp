// RT scheduling setup + introspection. Linux-only (Jetson/aarch64).
//
// enable_rt is ported from rtos_testing/cpp/grav_comp_test.cpp's enable_rt():
// mlockall + SCHED_FIFO, made best-effort (never throws) and extended with
// CPU affinity and a populated RtReport.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "kinova_lowlevel/rt_system.h"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

namespace kinova {

namespace {
const char* policy_name(int policy) {
  switch (policy) {
    case SCHED_FIFO:
      return "FIFO";
    case SCHED_RR:
      return "RR";
    case SCHED_OTHER:
      return "OTHER";
    default:
      return "UNKNOWN";
  }
}

// Held open for the process lifetime. Writing 0 to /dev/cpu_dma_latency places a
// PM-QoS request that forbids CPU idle states with non-zero exit latency (kills
// the ~5 ms 'c7' deep-idle wakeup hit) — the same thing cyclictest does. The QoS
// is released when the fd closes, so we keep it open until exit. Per-process: no
// global sysfs writes, no sudo (given a udev rule granting the device to our
// user/group). -1 = not pinned.
int g_cpu_dma_latency_fd = -1;
}  // namespace

RtReport enable_rt(const RtConfig& cfg) {
  RtReport rep;

  if (cfg.lock_memory) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
      rep.mlock_ok = true;
    } else {
      rep.note += "mlockall failed: ";
      rep.note += std::strerror(errno);
      rep.note += "; ";
    }
  }

  if (cfg.priority > 0) {
    struct sched_param p {};
    p.sched_priority = cfg.priority;
    if (sched_setscheduler(0, SCHED_FIFO, &p) != 0) {
      rep.note += "SCHED_FIFO failed: ";
      rep.note += std::strerror(errno);
      rep.note +=
          " (run with sudo or `setcap cap_sys_nice,cap_ipc_lock+ep`); ";
    }
  }

  if (cfg.cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cfg.cpu, &set);
    if (sched_setaffinity(0, sizeof(set), &set) != 0) {
      rep.note += "sched_setaffinity failed: ";
      rep.note += std::strerror(errno);
      rep.note += "; ";
    }
  }

  if (cfg.pin_cpu_latency && g_cpu_dma_latency_fd < 0) {
    int fd = ::open("/dev/cpu_dma_latency", O_WRONLY | O_CLOEXEC);
    if (fd >= 0) {
      const int32_t target_us = 0;
      if (::write(fd, &target_us, sizeof(target_us)) == sizeof(target_us)) {
        g_cpu_dma_latency_fd = fd;  // hold open for process lifetime
        rep.cpu_latency_pinned = true;
      } else {
        ::close(fd);
        rep.note += "cpu_dma_latency write failed: ";
        rep.note += std::strerror(errno);
        rep.note += "; ";
      }
    } else {
      rep.note += "cpu_dma_latency open failed: ";
      rep.note += std::strerror(errno);
      rep.note +=
          " (deep C-states NOT pinned; add a udev rule granting "
          "/dev/cpu_dma_latency to your user/group); ";
    }
  } else if (g_cpu_dma_latency_fd >= 0) {
    rep.cpu_latency_pinned = true;  // already pinned by a prior call
  }

  rep.policy = sched_getscheduler(0);
  struct sched_param gp {};
  if (sched_getparam(0, &gp) == 0) {
    rep.priority = gp.sched_priority;
  }
  rep.cpu = sched_getcpu();

  return rep;
}

ResourceUsage read_usage() {
  ResourceUsage u;
  struct rusage ru {};
  if (getrusage(RUSAGE_THREAD, &ru) == 0) {
    u.minflt = static_cast<uint64_t>(ru.ru_minflt);
    u.majflt = static_cast<uint64_t>(ru.ru_majflt);
    u.nvcsw = static_cast<uint64_t>(ru.ru_nvcsw);
    u.nivcsw = static_cast<uint64_t>(ru.ru_nivcsw);
  }
  return u;
}

std::string introspect() {
  const int policy = sched_getscheduler(0);
  struct sched_param gp {};
  int prio = -1;
  if (sched_getparam(0, &gp) == 0) prio = gp.sched_priority;
  const int cpu = sched_getcpu();
  const ResourceUsage u = read_usage();

  std::string out;
  out += "RT introspection:\n";
  out += "  policy:   ";
  out += policy_name(policy);
  out += "\n";
  out += "  priority: " + std::to_string(prio) + "\n";
  out += "  cpu:      " + std::to_string(cpu) + "\n";
  out += "  minflt:   " + std::to_string(u.minflt) + "\n";
  out += "  majflt:   " + std::to_string(u.majflt) + "\n";
  out += "  nvcsw:    " + std::to_string(u.nvcsw) + "\n";
  out += "  nivcsw:   " + std::to_string(u.nivcsw) + "\n";
  out += "  cpu_dma_latency: ";
  out += (g_cpu_dma_latency_fd >= 0) ? "pinned@0us\n" : "not pinned\n";
  return out;
}

}  // namespace kinova
