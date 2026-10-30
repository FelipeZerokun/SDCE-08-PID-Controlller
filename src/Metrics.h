#ifndef METRICS_H
#define METRICS_H
#include <algorithm>
#include <cmath>
#include <ostream>
struct Metrics {
  double duration=0, squared=0, maximum=0, rate_squared=0, saturated=0, compute_us=0, previous=0;
  int count=0;
  void Add(double e, double u, double dt, double us) {
    duration += dt; squared += e*e*dt; maximum = std::max(maximum, std::abs(e));
    if (count) rate_squared += (u-previous)*(u-previous)/dt;
    saturated += (std::abs(u) >= 0.999 ? dt : 0);
    compute_us += us; previous=u; ++count;
  }
  void Write(std::ostream& out) const {
    const double t = std::max(duration, 1e-12);
    out << duration << ',' << std::sqrt(squared/t) << ',' << maximum << ','
        << std::sqrt(rate_squared/t) << ',' << 100*saturated/t << ',' << compute_us/std::max(count,1);
  }
};
#endif
