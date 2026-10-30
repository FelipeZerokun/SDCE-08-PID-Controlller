#include "Controller.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
Controller::Controller(std::string name) : name_(std::move(name)) {
  if (name_ != "pid" && name_ != "mpc" && name_ != "lqr")
    throw std::invalid_argument("Controller must be pid, mpc or lqr");
}
void Controller::Reset() { mpc_.Reset(); estimate_ = {}; integral_ = 0; }
double Controller::Update(double cte, double speed, double dt) {
  if (name_ == "mpc") return mpc_.Update(cte, speed, dt);
  const double heading = estimate_.Update(cte, speed, dt);
  if (dt > 0.5) integral_ = 0;
  if (name_ == "pid") {
    // Time-aware PID, filtered D, conditional integration and bounded I contribution.
    // These are fresh baseline gains, not the old per-message legacy gains.
    constexpr double kp = 0.12, ki = 0.015, kd = 0.18;
    const double candidate = std::clamp(integral_ + cte * std::min(dt, 0.5), -10.0, 10.0);
    const double raw = -kp * cte - ki * candidate - kd * estimate_.rate;
    if (std::abs(raw) <= 1 || raw * cte > 0) integral_ = candidate;
    return std::clamp(-kp * cte - ki * integral_ - kd * estimate_.rate, -1.0, 1.0);
  }
  // Continuous-time infinite-horizon LQR: x=[e,psi], A=[0,v;0,0], B=[0;b].
  // b=v*max_angle/L, Q=diag(1,1), R=25. Closed-form stabilizing Riccati solution:
  // K_e=sqrt(Q_e/R), K_psi=sqrt((2*(v/b)*sqrt(Q_e*R)+Q_psi)/R).
  // v/b=L/max_angle, so the gains are constant for every positive speed.
  // At standstill there is no lateral controllability; this is its low-speed extension.
  constexpr double qe = 1, qpsi = 1, r = 25;
  const double ke = std::sqrt(qe / r);
  const double kpsi = std::sqrt((2 * MPC::wheelbase / MPC::max_angle * std::sqrt(qe*r) + qpsi) / r);
  // Clipping is outside the LQR optimization; constrained optimality is not claimed.
  return std::clamp(-ke * cte - kpsi * heading, -1.0, 1.0);
}
