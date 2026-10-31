#include "Controller.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

// Independent nonlinear plant, time-based delay, and zero-order-held commands.
// Optional recorded CSV supplies only sample intervals, never measured CTE responses.
int main(int argc, char** argv) {
  std::vector<double> intervals{0.06,0.08,0.10,0.07,0.18,0.09,0.08,0.12};
  if (argc > 1) {
    std::ifstream file(argv[1]); std::string line; std::getline(file,line);
    intervals.clear();
    while (std::getline(file,line)) {
      std::stringstream row(line); std::string field;
      for(int i=0;i<5;++i) std::getline(row,field,',');
      double dt=std::stod(field);
      if(dt>0 && dt<=0.5) intervals.push_back(dt);
    }
    if(intervals.empty()) { std::cerr << "No valid timing samples\n"; return 1; }
  }
  const double clock_scale=argc>2 ? std::stod(argv[2]) : 1.0;
  if(!std::isfinite(clock_scale)||clock_scale<0.1||clock_scale>2.0) return 1;
  bool passed=true;
  for(const std::string name : {"pid","lqr"}) for(double speed : {11.2,13.4}) for(double delay : {0.0,0.10,0.18}) for(double gain : {0.8,1.2}) {
    Controller mpc(name); double t=0,e=0.76,heading=0,actual=0,previous=0,squared=0,duration=0,maxe=0,saturated=0;
    std::deque<std::pair<double,double>> queue; double target=0;
    size_t i=0; double elapsed=0.02;
    while(t<30) {
      const double dt=intervals[i++%intervals.size()];
      const double u=mpc.Update(e,speed,elapsed);
      if(!std::isfinite(u) || std::abs(u)>1) return 1;
      if(std::abs(u-previous)>1.5*std::min(elapsed,0.1)+1e-9) {
        std::cerr << "Steering-rate constraint violated\n"; return 1;
      }
      queue.push_back({t+delay,u});
      // Keep plant integration much faster than control updates.
      const double plant_dt=dt*clock_scale;
      const int steps=int(std::ceil(plant_dt/0.005)); const double h=plant_dt/steps;
      for(int k=0;k<steps;++k) {
        while(!queue.empty() && queue.front().first<=t) { target=queue.front().second; queue.pop_front(); }
        actual+=(target-actual)*(1-std::exp(-h/0.08));
        const double curvature=t<3 ? 0 : 0.015*std::sin((t-3)*0.5);
        e+=speed*std::sin(heading)*h;
        heading+=(speed/2.8*std::tan(MPC::max_angle*gain*actual)-speed*curvature)*h;
        t+=h; maxe=std::max(maxe,std::abs(e));
        if(t>3) { squared+=e*e*h; duration+=h; saturated+=(std::abs(u)>=.999?h:0); }
      }
      previous=u; elapsed=dt;
    }
    double rms=std::sqrt(squared/duration);
    bool ok=rms<0.9 && maxe<2.0 && saturated/duration<0.05;
    passed &= ok;
    std::cout << name << " speed=" << speed << " delay=" << delay << " gain=" << gain << " rms=" << rms << " max=" << maxe << " saturated=" << saturated/duration << " " << (ok?"PASS":"FAIL") << '\n';
  }
  return passed?0:1;
}
