#ifndef STATE_ESTIMATE_H
#define STATE_ESTIMATE_H
#include <algorithm>
#include <cmath>
#include <stdexcept>
struct StateEstimate {
  bool initialized = false;
  double previous = 0, rate = 0;
  double Update(double cte, double speed, double dt) {
    if (!std::isfinite(cte) || !std::isfinite(speed) || !std::isfinite(dt) || speed < 0 || dt <= 0)
      throw std::invalid_argument("Expected finite CTE, nonnegative speed and positive dt");
    if (dt > 0.5) *this = {};
    if (initialized) rate += dt / (0.08 + dt) * ((cte - previous) / dt - rate);
    previous = cte;
    initialized = true;
    return speed > 1 ? std::clamp(rate / speed, -0.5, 0.5) : 0;
  }
};
#endif
