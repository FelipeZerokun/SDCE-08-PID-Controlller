#include "Controller.h"
#include "LQR.h"
#include <cmath>
#include <algorithm>
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
    const auto gains=DiscreteLQRGains(10,0.01);
    Check(std::abs(gains.error-std::sqrt(3.0/25.0))<0.01,"Discrete LQR approaches continuous gain at small dt");
    Check(std::abs(lqr.Update(1,10,0.02))<=0.03+1e-10,"LQR command rate limit");
    for(double speed : {1.0,11.2,22.4}) for(double dt : {0.05,0.1,0.2}) {
      const auto k=DiscreteLQRGains(speed,dt);
      const double a=speed*dt, b=speed/MPC::wheelbase*MPC::max_angle;
      const double x=0.5*speed*b*dt*dt,y=b*dt;
      const double trace=2-x*k.error-y*k.heading;
      const double determinant=1-x*k.error-y*k.heading+a*y*k.error;
      const double discriminant=trace*trace-4*determinant;
      const double radius=discriminant<0 ? std::sqrt(determinant) :
        std::max(std::abs((trace+std::sqrt(discriminant))/2),std::abs((trace-std::sqrt(discriminant))/2));
      Check(radius<1,"Unconstrained discrete LQR must stabilize its nominal model");
    }
    for(const std::string name : {"pid","lqr"}) {
      Controller rate_limited(name); double last=0;
      for(int i=0;i<200;++i) {
        const double dt=i%2 ? 0.08:0.18;
        const double u=rate_limited.Update(i%20<10?4:-4,13.4,dt);
        Check(std::abs(u-last)<=1.5*std::min(dt,0.1)+1e-9,"Command rate limit");
        last=u;
      }
      rate_limited.Reset();
      Check(std::abs(rate_limited.Update(0,13.4,0.02))<1e-12,"Reset clears response estimate");
    }
    Controller pid("pid");
    for (int i=0;i<500;++i) pid.Update(100,0,0.02);
    Check(std::abs(pid.Update(0,0,1))<1e-12,"PID pause clears windup and derivative");
    std::cout << "Controller checks passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
