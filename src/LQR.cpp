#include "LQR.h"
#include "MPC.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
LQRGains DiscreteLQRGains(double speed, double dt) {
  if(!std::isfinite(speed)||!std::isfinite(dt)||speed<=0||dt<=0)
    throw std::invalid_argument("LQR requires positive finite speed and timestep");
  const double a=speed*dt, b=speed/MPC::wheelbase*MPC::max_angle;
  const double x=0.5*speed*b*dt*dt, y=b*dt;
  double p=1,q=0,r=1;
  for(int i=0;i<3000;++i) {
    const double pb0=p*x+q*y, pb1=q*x+r*y;
    const double denominator=25+x*pb0+y*pb1;
    const double g0=pb0,g1=a*pb0+pb1;
    const double nextp=3+p-g0*g0/denominator;
    const double nextq=a*p+q-g0*g1/denominator;
    const double nextr=1+a*a*p+2*a*q+r-g1*g1/denominator;
    const double residual=std::max({std::abs(nextp-p),std::abs(nextq-q),std::abs(nextr-r)});
    if(residual < 1e-9*std::max({1.0,std::abs(p),std::abs(q),std::abs(r)}))
      return {g0/denominator,g1/denominator,residual};
    p=nextp; q=nextq; r=nextr;
  }
  throw std::runtime_error("Discrete Riccati iteration did not converge");
}
