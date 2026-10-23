#pragma once
// KORTEX build only. The bounded BaseCyclic round trip behind every cyclic call
// in KortexTransport; a header so tests/kortex_cyclic_test.cpp can drive it
// through the SDK's real RouterClient.
#include <BaseCyclicClientRpc.h>
#include <IRouterClient.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <string>

#include "kinova_lowlevel/transport.h"

namespace kinova {

// BaseCyclicClient::m_serviceVersion (private in the SDK; 1 in KORTEX 2.8.0).
constexpr uint32_t kBaseCyclicServiceVersion = 1;

// One BaseCyclic round trip (fct: a BaseCyclic::FunctionUids), bounded by
// `timeout`. BaseCyclicClient's Refresh, RefreshCommand and RefreshFeedback
// (KORTEX 2.8.0) are this same send + wait + parse, but they wait on a
// system_clock deadline (a clock step stretches or cuts it) and report expiry
// only as a runtime_error message. Here the wait is steady_clock and expiry is a
// TransportTimeout. A reply arriving late resolves its own abandoned message id,
// so it is never taken as a later call's reply. get() rethrows
// KDetailedException for an error frame from the arm; a failed sendto is a
// runtime_error from send() itself. Both stay fatal (see #96).
inline Kinova::Api::Frame kortex_cyclic_call(Kinova::Api::IRouterClient& router, uint32_t fct,
                                             const std::string& payload,
                                             std::chrono::milliseconds timeout) {
  const Kinova::Api::RouterClientSendOptions opts{false, 0, uint32_t(timeout.count())};
  std::future<Kinova::Api::Frame> reply =
      router.send(payload, kBaseCyclicServiceVersion, fct, 0, opts);
  if (reply.wait_for(timeout) != std::future_status::ready) {
    throw TransportTimeout("KortexTransport: no BaseCyclic reply within the cyclic timeout");
  }
  return reply.get();
}

}  // namespace kinova
