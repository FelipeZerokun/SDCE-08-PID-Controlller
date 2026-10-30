#ifndef MPC_H
#define MPC_H
#include <array>
#include "StateEstimate.h"
class MPC {
 public:
  static constexpr int horizon = 12;
  static constexpr double step = 0.1;
  static constexpr double wheelbase = 2.67;
  static constexpr double max_angle = 0.436332313;
  double Update(double cte, double speed_mps, double elapsed_seconds);
  void Reset();
 private:
  double Cost(const std::array<double, horizon>& u, double e, double heading, double speed) const;
  std::array<double, horizon> plan_{};
  StateEstimate estimate_;
  double previous_command_ = 0.0;
};
#endif
