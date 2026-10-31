#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "MPC.h"
#include "StateEstimate.h"
#include <string>

// All controllers receive identical CTE, speed (m/s), and elapsed time (seconds).
class Controller {
 public:
  explicit Controller(std::string name = "mpc");
  double Update(double cte, double speed, double dt);
  void Reset();
  const std::string& Name() const { return name_; }
 private:
  std::string name_;
  MPC mpc_;
  StateEstimate estimate_;
  double integral_ = 0;
  double previous_command_ = 0;
  double response_command_ = 0;
  double sample_step_ = 0.1;
};
#endif
