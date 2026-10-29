#include "kinova_lowlevel/arm_route.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace kinova {

std::string arm_route_problem(const std::string& ip) {
  sockaddr_in arm{};
  arm.sin_family = AF_INET;
  arm.sin_port = htons(10000);  // any port: only the route matters
  if (inet_pton(AF_INET, ip.c_str(), &arm.sin_addr) != 1)
    throw std::invalid_argument("the arm's address is not an IPv4 address: '" + ip + "'");

  // Connecting a UDP socket sends nothing; the kernel just picks the route and source address.
  const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) return std::string("route check: no socket: ") + std::strerror(errno);
  sockaddr_in src{};
  socklen_t len = sizeof src;
  int err = 0;
  if (connect(fd, reinterpret_cast<const sockaddr*>(&arm), sizeof arm) != 0 ||
      getsockname(fd, reinterpret_cast<sockaddr*>(&src), &len) != 0)
    err = errno;
  close(fd);
  if (err) return "no route to " + ip + " (" + std::strerror(err) + ")";

  ifaddrs* ifs = nullptr;
  if (getifaddrs(&ifs) != 0)
    return std::string("route check: no interface list: ") + std::strerror(errno);
  // Stays "" if the source address is on no interface (it changed meanwhile): connect() tries.
  std::string problem;
  for (const ifaddrs* a = ifs; a; a = a->ifa_next) {
    if (!a->ifa_addr || a->ifa_addr->sa_family != AF_INET || !a->ifa_netmask) continue;
    const in_addr_t addr = reinterpret_cast<const sockaddr_in*>(a->ifa_addr)->sin_addr.s_addr;
    if (addr != src.sin_addr.s_addr) continue;
    const in_addr_t mask = reinterpret_cast<const sockaddr_in*>(a->ifa_netmask)->sin_addr.s_addr;
    char src_str[INET_ADDRSTRLEN] = "?";
    inet_ntop(AF_INET, &src.sin_addr, src_str, sizeof src_str);
    if ((arm.sin_addr.s_addr & mask) != (addr & mask))
      problem = "no direct route to " + ip + ": it would leave via " + a->ifa_name + " (" +
                src_str + "), whose network doesn't include it";
    else if (!(a->ifa_flags & IFF_RUNNING))
      problem = "no link on " + std::string(a->ifa_name) + ", the interface for " + ip;
    break;
  }
  freeifaddrs(ifs);
  return problem;
}

}  // namespace kinova
