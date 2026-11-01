#ifndef MPC_H
#define MPC_H
#include <array>
#include "StateEstimate.h"
#include "Diagnostics.h"
class MPC {
 public:
  static constexpr int horizon = 12;
  static constexpr double step = 0.1;
  static constexpr double wheelbase = 2.67;
  static constexpr double max_angle = 0.436332313;
  static constexpr double steering_rate = 1.5; // normalized units / second
  double Update(double cte, double speed_mps, double elapsed_seconds);
  void Reset();
  const ControllerDiagnostics& Diagnostics() const { return diagnostics_; }
 private:
  ControllerDiagnostics diagnostics_;
  double Cost(const std::array<double, horizon>& u, double e, double heading, double speed) const;
  std::array<double, horizon> plan_{};
  StateEstimate estimate_;
  double previous_command_ = 0.0;

  double integral_bias_ = 0.0;
  double first_step_ = step;
};
#endif
