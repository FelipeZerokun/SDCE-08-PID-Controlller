#include "Controller.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
void Check(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
int main() {
  try {
    for (const std::string name : {"pid","mpc","lqr"}) {
      Controller a(name),b(name);
      const double u=a.Update(1,11,0.02);
      Check(u<0 && std::abs(u+b.Update(-1,11,0.02))<1e-6,"Sign and symmetry");
      a.Reset(); Check(std::abs(a.Update(1,11,0.02)-u)<1e-10,"Reset equivalence");
      bool rejected=false;
      try { a.Update(0,11,0); } catch (const std::invalid_argument&) { rejected=true; }
      Check(rejected,"Reject zero dt");
      a.Reset(); double e=1,heading=0;
      for (int i=0;i<1000;++i) {
        const double dt=i%2 ? 0.025:0.015;
        const double command=a.Update(e,11,dt);
        Check(std::isfinite(command)&&std::abs(command)<=1,"Output bounded");
        e+=11*std::sin(heading)*dt;
        heading+=11/2.8*std::tan(MPC::max_angle*command)*dt;
      }
      Check(std::abs(e)<0.1,"Straight-road recovery");
    }
    Controller lqr("lqr");
    Check(std::abs(lqr.Update(1,10,0.02)+0.2)<1e-10,"Closed-form LQR CTE gain");
    Controller pid("pid");
    for (int i=0;i<500;++i) pid.Update(100,0,0.02);
    Check(std::abs(pid.Update(0,0,1))<1e-12,"PID pause clears windup and derivative");
    std::cout << "Controller checks passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
