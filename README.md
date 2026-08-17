# Hybrid Electric Controls

Progressive controls sandbox for hybrid-electric propulsion. Each week stays separated so the models and control ideas remain easy to inspect and modify.

## Setup

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Week-by-week structure

### Week 1 — single inertia + PI speed control

Core equation:

`J * domega/dt = torque_cmd - torque_load - b * omega`

Files:
- `hyb_ctrl/motor.py`
- `hyb_ctrl/controllers.py`
- `examples/week1_speed_control.py`

Practice: speed step tracking, steady-state error, PI tuning, and a load-torque disturbance.

### Week 2 — two-inertia torsional drivetrain

States are motor speed, load speed, and shaft twist.

Files:
- `hyb_ctrl/two_inertia.py`
- `examples/week2_two_inertia.py`

Practice: coupling, torsional oscillation, natural modes, and disturbance transmission.

### Week 3 — state space, controllability, observability, modes

Files:
- `hyb_ctrl/state_space.py`
- `examples/week3_modes.py`

Practice: build A/B/C matrices, calculate eigenvalues/eigenvectors, and check controllability/observability ranks.

### Week 4 — cascaded control + discretization

Outer loop: speed -> torque reference.

Inner loop: torque/current tracking -> actuator command.

Files:
- `hyb_ctrl/cascaded.py`
- `hyb_ctrl/discrete.py`
- `examples/week4_cascade.py`

Practice: tune the inner loop faster than the outer loop, apply disturbances, and compare continuous versus discrete behavior.

### Week 5 — optimal control, estimation, constrained control

LQR:
- `hyb_ctrl/lqr.py`
- `examples/week5_lqr.py`

Kalman filter:
- `hyb_ctrl/kalman.py`
- `examples/week5_kalman.py`

MPC:
- `hyb_ctrl/mpc.py`
- `examples/week5_mpc.py`

Practice:
- tune Q/R in LQR and inspect the closed-loop eigenvalues;
- change process/measurement noise in the Kalman filter;
- change the MPC horizon and actuator limits and observe how the solution changes.

## Run examples

```bash
python examples/week1_speed_control.py
python examples/week2_two_inertia.py
python examples/week3_modes.py
python examples/week4_cascade.py
python examples/week5_lqr.py
python examples/week5_kalman.py
python examples/week5_mpc.py
```

## Where this goes next

The next layer is to combine these pieces into a hybrid-propulsion plant: electric-machine torque control inside a shaft-speed/power loop, then a slower supervisory power allocator with battery SOC and thermal constraints. From there the controller can be discretized and moved toward real hardware.
