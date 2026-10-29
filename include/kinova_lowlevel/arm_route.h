#pragma once
#include <string>
namespace kinova {
// Why the arm at `ip` can't be reached directly right now, or "" when it can. The arm sits on
// its own link, so `ip` must be on the network of the interface the kernel would send through,
// and that interface must have a link. Otherwise a KORTEX connect() leaves by the default
// route (wifi, say) and blocks ~127 s in TCP SYN retries. Reads the routing table and the
// interface list only: sends nothing, needs no privilege. Throws std::invalid_argument if `ip`
// is not an IPv4 address.
std::string arm_route_problem(const std::string& ip);
}  // namespace kinova
