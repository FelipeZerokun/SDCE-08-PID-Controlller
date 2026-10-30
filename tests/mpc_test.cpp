#include "MPC.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
void Check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
int main() {
  try {
    MPC a, b;
    Check(std::abs(a.Update(0, 10, 0.02)) < 1e-8, "Centered state must give zero steering");
    a.Reset();
    const double left = a.Update(1, 10, 0.02), right = b.Update(-1, 10, 0.02);
    Check(left < 0 && right > 0 && std::abs(left + right) < 1e-6, "Correction sign and symmetry");
    a.Reset();
    Check(std::abs(a.Update(1, 10, 0.02) - left) < 1e-8, "Reset must clear history");
    bool rejected = false;
    try { a.Update(std::numeric_limits<double>::quiet_NaN(), 10, 0.02); }
    catch (const std::invalid_argument&) { rejected = true; }
    Check(rejected, "Reject invalid input");
    // Independent nonlinear bicycle plant, slightly different wheelbase, curved road.
    // This is a numerical check, not evidence of a successful Unity simulator lap.
    for (double speed : {5.0, 11.0, 18.0}) {
      for (double road_curvature : {0.0, 0.01, -0.01}) {
        MPC mpc;
        double e = 1.0, heading = 0.0, squared = 0.0, maximum = 0.0;
        for (int i = 0; i < 800; ++i) {
          const double dt = i % 2 ? 0.025 : 0.015;
          const double u = mpc.Update(e, speed, dt);
          Check(std::isfinite(u) && std::abs(u) <= 1.0, "Steering bound");
          e += speed * std::sin(heading) * dt;
          heading += (speed / 2.8 * std::tan(MPC::max_angle * u) - speed * road_curvature) * dt;
          maximum = std::max(maximum, std::abs(e));
          if (i >= 400) squared += e * e;
        }
        const double rms = std::sqrt(squared / 400);
        std::cout << "speed=" << speed << " curvature=" << road_curvature << " RMS=" << rms << " max=" << maximum << '\n';
        Check(rms < 0.4 && maximum < 2.0, "Closed-loop tracking failed");
      }
    }
    std::cout << "All MPC checks passed\n";
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
