#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  /**
   * PID Coefficients
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  /**
   * PID Errors
   */
  p_error = 0;
  i_error = 0;
  d_error = 0;
  // For the derivative, I need the previous error
  prev_err = 0;
  // For the integrate, I need the sum of errors
  // sum_err = 0; NOT NEEDED
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  // Proportional error
  p_error = cte;
  // Derivative error
  d_error = cte - prev_err;
  prev_err = cte;
  // Integral error
  // sum_err += cte; NOT NEEDED
  // i_err = sum_err; NOT NEEDED
  i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double total_error = -p_error*Kp - d_error*Kd - i_error*Ki;
  return total_error;  // TODO: Add your total error calc here!
}
