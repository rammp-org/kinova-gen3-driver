#pragma once
#include <mutex>
#include <vector>
#include "kinova_lowlevel/interface/ports.h"
namespace kinova::interface {
class FakeBackend : public StreamPort, public ActionServerPort {
 public:
  void publish_state(const ArmState& s) override { std::lock_guard<std::mutex> l(m_); states_.push_back(s); }
  void publish_feedback(const GoalId&, const TrajectoryFeedback& f) override {
    std::lock_guard<std::mutex> l(m_); feedback_.push_back(f); }
  void settle(const GoalId& id, const TrajectoryResult& r) override {
    std::lock_guard<std::mutex> l(m_); results_.push_back({id, r}); }
  size_t state_count()   const { std::lock_guard<std::mutex> l(m_); return states_.size(); }
  size_t result_count()  const { std::lock_guard<std::mutex> l(m_); return results_.size(); }
  size_t feedback_count() const { std::lock_guard<std::mutex> l(m_); return feedback_.size(); }
  ArmState last_state()  const { std::lock_guard<std::mutex> l(m_); return states_.back(); }
  TrajectoryResult last_result() const { std::lock_guard<std::mutex> l(m_); return results_.back().second; }
  GoalId last_result_id() const { std::lock_guard<std::mutex> l(m_); return results_.back().first; }
  std::vector<std::pair<GoalId, TrajectoryResult>> all_results() const {
    std::lock_guard<std::mutex> l(m_); return results_; }
 private:
  mutable std::mutex m_;
  std::vector<ArmState> states_;
  std::vector<TrajectoryFeedback> feedback_;
  std::vector<std::pair<GoalId, TrajectoryResult>> results_;
};
}  // namespace kinova::interface
