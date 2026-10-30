#include "Controller.h"
#include "Metrics.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <deque>
#include <stdexcept>
int main(int argc, char** argv) {
  try {
    if (argc > 2) throw std::invalid_argument("Usage: compare_controllers [new-output-directory]");
    const std::filesystem::path directory = argc == 2 ? argv[1] : "outputs/comparison";
    if (std::filesystem::exists(directory)) throw std::runtime_error("Output directory already exists; choose a new name");
    std::filesystem::create_directories(directory);
    std::ofstream summary(directory / "summary.csv");
    summary << "controller,scenario,speed_mph,duration_s,rms_cte_m,max_cte_m,rms_steering_rate_per_s,saturation_percent,mean_compute_us\n";
    for (const std::string name : {"pid", "lqr", "mpc"}) {
      for (double mph : {15.0, 25.0, 40.0}) {
        for (const std::string scenario : {"straight", "curves", "bias", "delay"}) {
          Controller controller(name);
          Metrics metrics;
          const double speed = mph*0.44704, dt=0.02;
          double e=1, heading=0;
          std::deque<double> pending(5, 0); // 100 ms actuator delay in delay scenario only.
          std::ofstream trace(directory / (name + "_" + scenario + "_" + std::to_string(int(mph)) + ".csv"));
          trace << "time_s,cte_m,steering,applied_steering,curvature_per_m\n";
          for (int i=0; i<1000; ++i) {
            const double t=i*dt;
            const double curvature = (scenario == "curves" || scenario == "delay") && t>=3 ? 0.015*std::sin((t-3)*0.5) : 0;
            const auto start=std::chrono::steady_clock::now();
            const double u=controller.Update(e, speed, dt);
            const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            if (!std::isfinite(u) || std::abs(u)>1) throw std::runtime_error("Invalid controller output");
            double applied=u;
            if (scenario == "delay") { pending.push_back(u); applied=pending.front(); pending.pop_front(); }
            if (scenario == "bias") applied=std::clamp(applied+0.025,-1.0,1.0);
            metrics.Add(e,u,dt,us);
            trace << t << ',' << e << ',' << u << ',' << applied << ',' << curvature << '\n';
            // Nonlinear plant with wheelbase mismatch; no controller sees curvature.
            e += speed*std::sin(heading)*dt;
            heading += (speed/2.8*std::tan(MPC::max_angle*applied)-speed*curvature)*dt;
          }
          summary << name << ',' << scenario << ',' << mph << ','; metrics.Write(summary); summary << '\n';
          std::cout << name << ' ' << scenario << ' ' << mph << "mph RMS=" << std::sqrt(metrics.squared/metrics.duration) << '\n';
          if (!trace) throw std::runtime_error("Could not write trace");
        }
      }
    }
    if (!summary) throw std::runtime_error("Could not write summary");
    std::cout << "Synthetic results saved to " << directory << "; not Unity lap results.\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
