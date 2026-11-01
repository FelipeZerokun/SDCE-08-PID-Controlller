#include "MPC.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace {
double Clamp(double x, double lo, double hi) { return std::max(lo, std::min(x, hi)); }
}
void MPC::Reset() {
  diagnostics_ = {};
  plan_.fill(0.0);
  estimate_ = {};
  previous_command_ = integral_bias_ = 0.0;
  first_step_ = step;
}
double MPC::Cost(const std::array<double, horizon>& u, double e, double heading, double speed) const {
  // Predict through a conservative 300 ms response allowance before new actions.
  // This covers sampling/filter/actuator lag approximately; it is not an identified delay.
  const double latency=0.30;
  const double held_yaw=speed/wheelbase*max_angle*previous_command_;
  e += speed*heading*latency + 0.5*speed*held_yaw*latency*latency;
  heading += held_yaw*latency;
  double cost = 0.0, previous = previous_command_;
  for (double command : u) {
    // Linear bicycle model: locally straight road, constant speed, small angles.
    // Positive steering increases positive CTE, consistent with legacy PID feedback.
    const double yaw_rate = speed / wheelbase * max_angle * command;
    e += speed * heading * step + 0.5 * speed * yaw_rate * step * step;
    heading += yaw_rate * step;
    const double change = command - previous;
    cost += 6.0 * e * e + 1.0 * heading * heading
            + 25.0 * command * command + 20.0 * change * change;
    previous = command;
  }
  // Continuous LQR terminal value for Q=diag(6,1), R=25, scaled to the stage sum.
  // A terminal heuristic for this sampled, constrained problem, not a stability proof.
  const double v=std::max(speed,1.0), b=v/wheelbase*max_angle;
  const double kh=std::sqrt((2.0*std::sqrt(150.0)*wheelbase/max_angle+1.0)/25.0);
  const double p12=std::sqrt(150.0)/b, p22=25.0*kh/b, p11=std::sqrt(150.0)*kh/v;
  return cost + (p11*e*e + 2*p12*e*heading + p22*heading*heading)/step;
}
double MPC::Update(double cte, double speed, double dt) {
  if (!std::isfinite(cte) || !std::isfinite(speed) || !std::isfinite(dt) || speed < 0.0 || dt <= 0.0)
    throw std::invalid_argument("MPC requires finite CTE, nonnegative speed and positive dt");
  if (dt > 0.5) Reset();  // Pause invalidates the derivative history.
  const double heading = estimate_.Update(cte, speed, dt);
  // Slow, bounded integral trim reduces offsets without aggressive steering gains.
  // Freeze at standstill and near amplitude saturation; clamp prevents unbounded windup.
  if(speed>1.0 && std::abs(previous_command_)<0.95)
    integral_bias_=Clamp(integral_bias_+0.1*cte*std::min(dt,0.1),-1.0,1.0);
  cte += integral_bias_;
  first_step_ = std::min(dt, step); // do not allow a large jump after a late packet

  diagnostics_ = {};
  diagnostics_.cte_rate=estimate_.rate; diagnostics_.heading=heading;
  diagnostics_.previous_command=previous_command_;
  diagnostics_.lower=std::max(-1.0,previous_command_-steering_rate*first_step_);
  diagnostics_.upper=std::min(1.0,previous_command_+steering_rate*first_step_);
  diagnostics_.mpc_bias=integral_bias_; diagnostics_.model_step=step;
  const double held_yaw=speed/wheelbase*max_angle*previous_command_;
  diagnostics_.predicted_error=cte+speed*heading*0.30+0.5*speed*held_yaw*0.30*0.30;
  diagnostics_.predicted_heading=heading+held_yaw*0.30;

  // Resample the previous plan at elapsed time; hold its tail for a warm start.
  const auto old = plan_;
  for (int k = 0; k < horizon; ++k) {
    const double index = std::min(k + dt / step, double(horizon - 1));
    const int i = int(index);
    plan_[k] = old[i] + (index - i) * (old[std::min(i + 1, horizon - 1)] - old[i]);
  }
  // Make the warm start feasible for amplitude AND successive-command rate limits.
  double previous = previous_command_;
  for (int k=0;k<horizon;++k) {
    const double change = steering_rate*(k==0 ? first_step_ : step);
    plan_[k] = Clamp(plan_[k], std::max(-1.0,previous-change), std::min(1.0,previous+change));
    previous=plan_[k];
  }
  // Convex quadratic cost + amplitude and adjacent-command constraints. Each coordinate has an exact minimum.
  // Coupled rate constraints can stall coordinate descent at a nonglobal solution.
  // Every iterate remains feasible; no optimality or convergence guarantee is claimed.
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
      const double before = k==0 ? previous_command_ : plan_[k-1];
      const double change = steering_rate*(k==0 ? first_step_ : step);
      double lower=std::max(-1.0,before-change), upper=std::min(1.0,before+change);
      if(k+1<horizon) {
        lower=std::max(lower,plan_[k+1]-steering_rate*step);
        upper=std::min(upper,plan_[k+1]+steering_rate*step);
      }
      plan_[k] = Clamp(original - (plus - minus) / (2.0 * curvature), lower, upper);
      largest_change = std::max(largest_change, std::abs(plan_[k] - original));
    }
    diagnostics_.mpc_sweeps=sweep+1;
    diagnostics_.mpc_last_change=largest_change;
    if (largest_change < 1e-5) break;
  }
  previous_command_ = plan_[0]; // Apply only the first action, then measure again.
  return previous_command_;
}
