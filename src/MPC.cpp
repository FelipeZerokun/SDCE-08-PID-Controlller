#include "MPC.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace {
double Clamp(double x, double lo, double hi) { return std::max(lo, std::min(x, hi)); }
}
void MPC::Reset() {
  plan_.fill(0.0);
  estimate_ = {};
  previous_command_ = 0.0;
}
double MPC::Cost(const std::array<double, horizon>& u, double e, double heading, double speed) const {
  double cost = 0.0, previous = previous_command_;
  for (double command : u) {
    // Linear bicycle model: locally straight road, constant speed, small angles.
    // Positive steering increases positive CTE, consistent with legacy PID feedback.
    const double yaw_rate = speed / wheelbase * max_angle * command;
    e += speed * heading * step + 0.5 * speed * yaw_rate * step * step;
    heading += yaw_rate * step;
    const double change = command - previous;
    cost += 5.0 * e * e + 2.0 * heading * heading
            + 0.5 * command * command + 8.0 * change * change;
    previous = command;
  }
  return cost + 10.0 * e * e + 5.0 * heading * heading;
}
double MPC::Update(double cte, double speed, double dt) {
  if (!std::isfinite(cte) || !std::isfinite(speed) || !std::isfinite(dt) || speed < 0.0 || dt <= 0.0)
    throw std::invalid_argument("MPC requires finite CTE, nonnegative speed and positive dt");
  if (dt > 0.5) Reset();  // Pause invalidates the derivative history.
  const double heading = estimate_.Update(cte, speed, dt);
  // Resample the previous plan at elapsed time; hold its tail for a warm start.
  const auto old = plan_;
  for (int k = 0; k < horizon; ++k) {
    const double index = std::min(k + dt / step, double(horizon - 1));
    const int i = int(index);
    plan_[k] = old[i] + (index - i) * (old[std::min(i + 1, horizon - 1)] - old[i]);
  }
  // Convex quadratic cost + box constraints. Each coordinate has an exact minimum.
  // Fixed iteration budget gives an approximate solution without solver dependencies.
  for (int sweep = 0; sweep < 80; ++sweep) {
    double largest_change = 0.0;
    for (int k = 0; k < horizon; ++k) {
      const double original = plan_[k];
      const double center = Cost(plan_, cte, heading, speed);
      plan_[k] = original + 1.0;
      const double plus = Cost(plan_, cte, heading, speed);
      plan_[k] = original - 1.0;
      const double minus = Cost(plan_, cte, heading, speed);
      const double curvature = plus + minus - 2.0 * center;
      plan_[k] = Clamp(original - (plus - minus) / (2.0 * curvature), -1.0, 1.0);
      largest_change = std::max(largest_change, std::abs(plan_[k] - original));
    }
    if (largest_change < 1e-5) break;
  }
  previous_command_ = plan_[0]; // Apply only the first action, then measure again.
  return previous_command_;
}
