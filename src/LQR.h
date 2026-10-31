#ifndef LQR_H
#define LQR_H
struct LQRGains { double error, heading, residual; };
// Discrete bicycle model; Q=diag(3,1), R=25. Throws if Riccati iteration fails.
LQRGains DiscreteLQRGains(double speed, double dt);
#endif
