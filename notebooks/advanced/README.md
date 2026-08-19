# Advanced Controls Track

These notebooks sit beside the simpler Week 1–5 primers. The primers explain the concepts; the advanced track forces engineering tradeoffs and failure analysis.

## Week 1 — Motor control under actuator limits
- PI with torque saturation
- integrator windup
- anti-windup
- load disturbances
- performance metrics and gain tradeoffs

## Week 2 — Flexible drivetrain
- torsional resonance
- Bode inspection before tuning
- controller interaction with structural modes
- speed tracking versus shaft-twist limits

## Week 3 — State-space engineering
- modal decomposition and participation
- sensor placement
- actuator placement
- controllability/observability rank versus conditioning
- PBH interpretation

## Week 4 — Cascades and digital implementation
- finite inner-loop bandwidth
- actuator rate limits
- bandwidth separation
- sample-rate sensitivity
- computational-delay exercises

## Week 5 — LQR, Kalman, LQG, and MPC
- one common plant for every method
- estimator noise
- hidden-state reconstruction
- model mismatch
- actuator/state constraints
- clipped LQR versus constrained MPC

## How to work through them

Do not just run every cell. Before each sweep or disturbance case, write down what you expect to happen and why. Afterward, explain any disagreement between the prediction and simulation.

Every notebook ends with an engineering deliverable. Treat that deliverable as the real assignment.

Weeks 6 onward transition from generic control plants to the hybrid-electric propulsion stack: hybrid supervisor, DC link, DC/DC converter, PMSM/FOC, inverter modulation, embedded implementation, and MIL/SIL/HIL.
