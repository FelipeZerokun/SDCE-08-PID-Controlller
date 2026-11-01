#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H
#include <cmath>
#include <limits>
#include <ostream>

// Snapshot only: no diagnostic value feeds back into control. NaN means unavailable.
struct ControllerDiagnostics {
  static constexpr double missing = std::numeric_limits<double>::quiet_NaN();
  double cte_rate=missing, heading=missing, previous_command=missing;
  double response_command=missing, predicted_error=missing, predicted_heading=missing;
  double predicted_rate=missing, raw_command=missing, lower=missing, upper=missing;
  double integral=missing, kp=missing, ki=missing, kd=missing;
  double lqr_error_gain=missing, lqr_heading_gain=missing, model_step=missing;
  double mpc_bias=missing, mpc_sweeps=missing, mpc_last_change=missing;

  static void Header(std::ostream& out) {
    out << ",cte_rate_mps,heading_est_rad,previous_command,response_command_est"
           ",predicted_cte_m,predicted_heading_rad,predicted_cte_rate_mps"
           ",raw_steering,steering_lower,steering_upper,pid_integral_m_s"
           ",pid_kp,pid_ki,pid_kd,lqr_error_gain,lqr_heading_gain,model_step_s"
           ",mpc_bias_m,mpc_sweeps,mpc_last_change";
  }
  void Write(std::ostream& out) const {
    for (double value : {cte_rate,heading,previous_command,response_command,
         predicted_error,predicted_heading,predicted_rate,raw_command,lower,upper,
         integral,kp,ki,kd,lqr_error_gain,lqr_heading_gain,model_step,
         mpc_bias,mpc_sweeps,mpc_last_change}) {
      out << ',';
      if (std::isfinite(value)) out << value;
    }
  }
};
#endif
