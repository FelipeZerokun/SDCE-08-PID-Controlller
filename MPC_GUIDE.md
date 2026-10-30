# MPC implementation guide

See [PLATFORM_SETUP.md](PLATFORM_SETUP.md) for Windows/Ubuntu builds,
[CONTROLLERS.md](CONTROLLERS.md) for the controller comparison, and
[MPC_RETEST.md](MPC_RETEST.md) for revision 2 and the next Ubuntu experiment.

## Read the code in this order

1. `src/simulator_session.cpp`: CTE and speed, mph-to-m/s conversion, elapsed time.
2. `src/MPC.cpp`, `Update`: filtered heading estimate, bounded integral trim, warm start.
3. `Cost`: response allowance, predicted trajectory, stage and terminal costs.
4. Coordinate updates: choose a feasible steering sequence.
5. Return only `plan_[0]`, measure again, and repeat (receding horizon control).

## Model

The state is lateral error e and heading error psi. Normalized steering u is in [-1,1].
The model assumes small angles, constant measured speed v over the prediction, and a
locally straight reference. Wheelbase 2.67 m and a maximum angle of 25 degrees remain
starting assumptions, not measured parameters of your installed simulator.

```text
r = (v / L) * delta_max * u
e_next = e + v * psi * h + 0.5 * v * r * h^2
psi_next = psi + r * h
```

Twelve prediction steps of h=0.1 s cover 1.2 s after the 0.30 s response allowance.
Heading is estimated from filtered CTE velocity divided by speed, clipped to +/-0.5
radians; below 1 m/s it is set to zero. First sample assumes zero heading. The
shared derivative filter time constant remains 0.08 s. There is no heading sensor,
curvature measurement, or preview. Reconnect when manually resetting the car.

## Objective and constraints (revision 2)

Stage cost: `6*e^2 + psi^2 + 25*u^2 + 20*(u-previous_u)^2`.
Before prediction, e is shifted by the slow bounded integral trim, and the state is
propagated with the held previous command for the response allowance.

Terminal cost: `x^T P x / h`, using the continuous-time Riccati solution for the
linear bicycle model with Q=diag(6,1), R=25. With b=v*delta_max/L:

```text
Kpsi = sqrt((2*sqrt(150)*L/delta_max + 1)/25)
P12  = sqrt(150)/b
P22  = 25*Kpsi/b
P11  = sqrt(150)*Kpsi/v
```

For this terminal calculation v is floored at 1 m/s to avoid division by zero.
This matrix is positive definite; it is not a formal terminal-set/stability guarantee
for the discrete constrained controller with estimated state and integral trim.

All predicted commands obey amplitude and rate bounds. The warm start is projected
into a feasible sequence. Each coordinate update is bounded by BOTH neighbors so
those constraints stay satisfied. Three cost evaluations recover the quadratic
minimum along one coordinate. At most 80 sweeps are used. With coupled constraints,
coordinate descent may stall at a nonglobal solution; feasibility does not imply
optimality. A dedicated QP solver is a possible later improvement.

Throttle is a separate proportional controller, identical for PID, LQR, and MPC.
No acceleration planning, lane constraints, or path preview is implemented.

## Experiments

Start with the revision-2 Ubuntu retest procedure. Once laps are repeatable, change
one weight or the response allowance at a time and rerun both numerical and simulator
checks. Review steering oscillation and saturation alongside RMS CTE and lap completion.
A lower RMS on an early failed run cannot establish a better controller.

Background: [MPC theory](https://www.do-mpc.com/en/latest/theory_mpc.html) and
[LQR terminal values](https://underactuated.mit.edu/lqr.html).
