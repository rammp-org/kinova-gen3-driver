// KORTEX build only (-DKINOVA_ENABLE_KORTEX=ON). Drives kortex_cyclic_call()
// through the SDK's real RouterClient over a fake link, so the lost-frame
// handling is checked against KORTEX itself, not against a model of it.
#include "kortex_cyclic.h"

#include <BaseCyclic.pb.h>
#include <HeaderInfo.h>
#include <ITransportClient.h>
#include <KDetailedException.h>
#include <RouterClient.h>
#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace k_api = Kinova::Api;
using namespace kinova;
using namespace std::chrono;

namespace {

// Stands in for the arm's UDP endpoint. Every request is kept; a reply is
// delivered from a separate thread, as KORTEX's own receive thread would.
class FakeLink : public k_api::ITransportClient {
 public:
  FakeLink() : worker_([this] { deliver(); }) {}
  ~FakeLink() override {
    {
      std::lock_guard<std::mutex> l(m_);
      done_ = true;
    }
    cv_.notify_all();
    worker_.join();
  }

  bool connect(std::string, uint32_t) override { return true; }
  void disconnect() override {}
  void send(const char* buf, uint32_t size) override {
    if (fail_sends) throw std::runtime_error("fake sendto failure");
    k_api::Frame f;
    ASSERT_TRUE(f.ParseFromArray(buf, int(size)));
    std::lock_guard<std::mutex> l(m_);
    requests.push_back(f);
  }
  void onMessage(std::function<void(const char*, uint32_t)> cb) override { on_msg_ = cb; }
  char* getTxBuffer(uint32_t const& n) override {
    tx_.resize(n);
    return tx_.data();
  }
  size_t getMaxTxBufferSize() override { return 65507; }
  void getHostAddress(std::string& host, uint32_t& port) override {
    host = "fake";
    port = 10001;
  }

  // Answer request `i` with feedback stamped `frame_id`, or with an error frame.
  void reply(size_t i, uint32_t frame_id, bool error = false) {
    k_api::Frame r;
    {
      std::lock_guard<std::mutex> l(m_);
      k_api::HeaderInfo h(requests.at(i).header());
      h.m_frameInfo.frameType = k_api::MSG_FRAME_RESPONSE;
      if (error) {
        h.m_frameInfo.errorCode = k_api::ERROR_DEVICE;
        h.m_frameInfo.errorSubCode = k_api::WRONG_SERVOING_MODE;
      }
      k_api::BaseCyclic::Feedback fb;
      fb.set_frame_id(frame_id);
      r.set_payload(fb.SerializeAsString());
      h.m_payloadInfo.payloadLength = uint32_t(r.payload().size());
      h.fillHeader(r.mutable_header());
      outbox_.push_back(r.SerializeAsString());
    }
    cv_.notify_all();
  }

  size_t count() {
    std::lock_guard<std::mutex> l(m_);
    return requests.size();
  }
  // Wait (bounded) until `n` requests have been sent.
  bool wait_requests(size_t n) {
    for (int k = 0; k < 1000 && count() < n; ++k) std::this_thread::sleep_for(milliseconds(1));
    return count() >= n;
  }

  std::vector<k_api::Frame> requests;
  bool fail_sends = false;

 private:
  void deliver() {
    std::unique_lock<std::mutex> l(m_);
    while (true) {
      cv_.wait(l, [this] { return done_ || !outbox_.empty(); });
      if (done_) return;
      std::string bytes = std::move(outbox_.front());
      outbox_.pop_front();
      l.unlock();
      on_msg_(bytes.data(), uint32_t(bytes.size()));
      l.lock();
    }
  }

  std::function<void(const char*, uint32_t)> on_msg_;
  std::vector<char> tx_;
  std::mutex m_;
  std::condition_variable cv_;
  std::deque<std::string> outbox_;
  bool done_ = false;
  std::thread worker_;  // last: started after everything it uses
};

struct Rig {
  FakeLink link;
  k_api::RouterClient router{&link, [](k_api::KError) {}};
};

uint32_t frame_id_of(const k_api::Frame& f) {
  k_api::BaseCyclic::Feedback fb;
  EXPECT_TRUE(fb.ParseFromString(f.payload()));
  return fb.frame_id();
}

const std::string kCmd = k_api::BaseCyclic::Command().SerializeAsString();

}  // namespace

// A reply is returned; the request is a BaseCyclic Refresh as KORTEX's own
// BaseCyclicClient::Refresh would have sent it.
TEST(KortexCyclic, ReplyIsReturned) {
  Rig r;
  std::thread arm([&] {
    ASSERT_TRUE(r.link.wait_requests(1));
    r.link.reply(0, 7);
  });
  const k_api::Frame f =
      kortex_cyclic_call(r.router, k_api::BaseCyclic::eUidRefresh, kCmd, milliseconds(500));
  arm.join();
  EXPECT_EQ(frame_id_of(f), 7u);
  k_api::HeaderInfo h(r.link.requests.at(0).header());
  EXPECT_EQ(h.m_serviceInfo.functionUid, uint32_t(k_api::BaseCyclic::eUidRefresh));
  EXPECT_EQ(h.m_serviceInfo.serviceVersion, kBaseCyclicServiceVersion);
}

// A lost reply is a TransportTimeout, raised at the bound, not after KORTEX's
// 3000 ms default.
TEST(KortexCyclic, LostReplyIsATimeoutAtTheBound) {
  Rig r;
  const auto t0 = steady_clock::now();
  EXPECT_THROW(kortex_cyclic_call(r.router, k_api::BaseCyclic::eUidRefresh, kCmd, milliseconds(20)),
               TransportTimeout);
  const auto ms = duration_cast<milliseconds>(steady_clock::now() - t0).count();
  EXPECT_GE(ms, 20);
  EXPECT_LT(ms, 200);
}

// A reply that arrives after its call timed out is not taken as the next call's
// reply: the next call gets its own.
TEST(KortexCyclic, LateReplyIsNotTakenByTheNextCall) {
  Rig r;
  EXPECT_THROW(kortex_cyclic_call(r.router, k_api::BaseCyclic::eUidRefresh, kCmd, milliseconds(20)),
               TransportTimeout);
  std::thread arm([&] {
    ASSERT_TRUE(r.link.wait_requests(2));
    r.link.reply(0, 1);  // the stale one first
    r.link.reply(1, 2);
  });
  const k_api::Frame f =
      kortex_cyclic_call(r.router, k_api::BaseCyclic::eUidRefresh, kCmd, milliseconds(500));
  arm.join();
  EXPECT_EQ(frame_id_of(f), 2u);
}

// An error frame from the arm is ArmRejected (the KDetailedException
// classified at the boundary), not a timeout.
TEST(KortexCyclic, ErrorFrameIsArmRejected) {
  Rig r;
  std::thread arm([&] {
    ASSERT_TRUE(r.link.wait_requests(1));
    r.link.reply(0, 0, /*error=*/true);
  });
  EXPECT_THROW(
      kortex_cyclic_call(r.router, k_api::BaseCyclic::eUidRefresh, kCmd, milliseconds(500)),
      ArmRejected);
  arm.join();
}

// A failed local send (sendto error) is TransportClientUdp's own runtime_error,
// passed straight through RouterClient::send. On a congested NIC that is a lost
// frame, so it is reported as one: a TransportTimeout carrying the sendto text,
// survived like any other miss (#96). The router stays usable after it: the
// next call is not blocked by the send that threw.
TEST(KortexCyclic, FailedLocalSendIsALostFrameAndTheRouterStaysUsable) {
  Rig r;
  r.link.fail_sends = true;
  try {
    kortex_cyclic_call(r.router, k_api::BaseCyclic::eUidRefresh, kCmd, milliseconds(20));
    FAIL() << "expected the send failure";
  } catch (const TransportTimeout& e) {
    EXPECT_NE(std::string(e.what()).find("fake sendto failure"), std::string::npos) << e.what();
  }
  r.link.fail_sends = false;
  std::thread arm([&] {
    ASSERT_TRUE(r.link.wait_requests(1));
    r.link.reply(0, 3);
  });
  const k_api::Frame f =
      kortex_cyclic_call(r.router, k_api::BaseCyclic::eUidRefresh, kCmd, milliseconds(500));
  arm.join();
  EXPECT_EQ(frame_id_of(f), 3u);
}
