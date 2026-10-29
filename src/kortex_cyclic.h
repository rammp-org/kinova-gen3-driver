#pragma once
// KORTEX build only. The bounded BaseCyclic round trip behind every cyclic call
// in KortexTransport; a header so tests/kortex_cyclic_test.cpp can drive it
// through the SDK's real RouterClient.
#include <BaseCyclicClientRpc.h>
#include <IRouterClient.h>
#include <KDetailedException.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <stdexcept>
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
// so it is never taken as a later call's reply. An error frame from the arm
// (get() rethrows KDetailedException) is ArmRejected. A failed sendto (a
// std::runtime_error from send() itself, TransportClientUdp's) is a lost frame on
// a congested NIC, so it is a TransportTimeout like any other miss (#96).
inline Kinova::Api::Frame kortex_cyclic_call(Kinova::Api::IRouterClient& router, uint32_t fct,
                                             const std::string& payload,
                                             std::chrono::milliseconds timeout) {
  const Kinova::Api::RouterClientSendOptions opts{false, 0, uint32_t(timeout.count())};
  std::future<Kinova::Api::Frame> reply;
  try {
    reply = router.send(payload, kBaseCyclicServiceVersion, fct, 0, opts);
  } catch (const Kinova::Api::KDetailedException& e) {
    // No transmit buffer, or the frame would not serialize: a local fault, not
    // the arm's answer, but just as non-transient as one.
    throw ArmRejected(std::string("KortexTransport: BaseCyclic send failed: ") + e.what());
  } catch (const std::runtime_error& e) {
    throw TransportTimeout(std::string("KortexTransport: BaseCyclic send failed: ") + e.what());
  }
  if (reply.wait_for(timeout) != std::future_status::ready) {
    throw TransportTimeout("KortexTransport: no BaseCyclic reply within the cyclic timeout");
  }
  try {
    return reply.get();
  } catch (const Kinova::Api::KDetailedException& e) {
    throw ArmRejected(std::string("KortexTransport: arm rejected the cyclic call: ") + e.what());
  }
}

}  // namespace kinova
