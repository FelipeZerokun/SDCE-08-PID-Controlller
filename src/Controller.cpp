#include "Controller.h"
#include "LQR.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
Controller::Controller(std::string name) : name_(std::move(name)) {
  if (name_ != "pid" && name_ != "mpc" && name_ != "lqr")
    throw std::invalid_argument("Controller must be pid, mpc or lqr");
}
void Controller::Reset() {
  mpc_.Reset(); estimate_ = {}; integral_ = previous_command_ = response_command_ = 0; sample_step_=0.1;
}
double Controller::Update(double cte, double speed, double dt) {
  if (name_ == "mpc") return mpc_.Update(cte, speed, dt);
  if (!std::isfinite(cte)||!std::isfinite(speed)||!std::isfinite(dt)||speed<0||dt<=0)
    throw std::invalid_argument("Expected finite CTE, nonnegative speed and positive dt");
  if (dt > 0.5) Reset();
  const double heading = estimate_.Update(cte, speed, dt);
  response_command_ += (previous_command_-response_command_)*(1-std::exp(-dt/0.15));
  const double limit=1.5*std::min(dt,0.1);
  const double lower=std::max(-1.0,previous_command_-limit);
  const double upper=std::min(1.0,previous_command_+limit);
  double command;
  if (name_ == "pid") {
    // Reduce derivative gain above 11.2 m/s; do not amplify it at low speed.
    const double scale=11.2/std::max(speed,11.2);
    const double kp=0.25,ki=0.003,kd=0.12*scale;
    const double candidate=speed>1 ? std::clamp(integral_+cte*std::min(dt,0.1),-10.0,10.0) : integral_;
    // Predictive PID wrapper: a short held-command prediction compensates some
    // feedback lag. This adds a model assumption to an otherwise error-based PID.
    const double response=0.35;
    const double acceleration=speed*speed/MPC::wheelbase*MPC::max_angle*response_command_;
    const double predicted_error=cte+estimate_.rate*response+0.5*acceleration*response*response;
    const double predicted_rate=estimate_.rate+acceleration*response;
    const double raw=-kp*predicted_error-ki*candidate-kd*predicted_rate;
    const double limited=std::clamp(raw,lower,upper);
    // Conditional integration considers both actuator amplitude AND rate limiting.
    if(std::abs(raw-limited)<1e-10 || (raw-limited)*cte>0) integral_=candidate;
    command=std::clamp(-kp*predicted_error-ki*integral_-kd*predicted_rate,lower,upper);
  } else {
    // Smooth sample-time scheduling so packet jitter does not abruptly retune K.
    sample_step_ += 0.15*(std::clamp(dt,0.05,0.2)-sample_step_);
    const auto gains=DiscreteLQRGains(std::max(speed,1.0),sample_step_);
    // Approximate lag allowance, not measured simulator delay. Plain LQR remains
    // unconstrained; amplitude/rate limits below are external actuator protections.
    constexpr double response=0.30;
    const double yaw=speed/MPC::wheelbase*MPC::max_angle*response_command_;
    const double predicted_error=cte+speed*heading*response+0.5*speed*yaw*response*response;
    const double predicted_heading=heading+yaw*response;
    command=std::clamp(-gains.error*predicted_error-gains.heading*predicted_heading,lower,upper);
  }
  previous_command_=command;
  return command;
}
