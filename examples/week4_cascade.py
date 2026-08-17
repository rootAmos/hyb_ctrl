import numpy as np
import matplotlib.pyplot as plt

from hyb_ctrl.cascaded import CascadedSpeedTorqueController
from hyb_ctrl.controllers import PIController
from hyb_ctrl.motor import SingleInertiaMotor


def main() -> None:
    dt = 0.0005
    time = np.arange(0.0, 3.0, dt)

    plant = SingleInertiaMotor(J=0.05, b=0.02)
    controller = CascadedSpeedTorqueController(
        speed_loop=PIController(kp=0.25, ki=1.5, u_min=-8.0, u_max=8.0),
        torque_loop=PIController(kp=6.0, ki=120.0, u_min=-15.0, u_max=15.0),
    )

    speed_ref = 100.0
    torque_actual = 0.0
    tau_actuator = 0.01

    omega_hist = np.zeros_like(time)
    torque_ref_hist = np.zeros_like(time)
    torque_hist = np.zeros_like(time)

    for k, t in enumerate(time):
        load = 2.0 if t >= 1.5 else 0.0
        torque_ref, actuator_cmd = controller.update(
            speed_ref,
            plant.omega,
            torque_actual,
            dt_outer=dt,
            dt_inner=dt,
        )

        # Simple first-order actuator/current-loop plant.
        torque_actual += (actuator_cmd - torque_actual) * dt / tau_actuator
        plant.step(torque_actual, load, dt)

        omega_hist[k] = plant.omega
        torque_ref_hist[k] = torque_ref
        torque_hist[k] = torque_actual

    fig, ax = plt.subplots()
    ax.plot(time, omega_hist, label="shaft speed")
    ax.axhline(speed_ref, linestyle="--", label="speed reference")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [rad/s]")
    ax.set_title("Week 4: cascaded torque + speed loops")
    ax.grid(True)
    ax.legend()

    fig2, ax2 = plt.subplots()
    ax2.plot(time, torque_ref_hist, label="torque reference")
    ax2.plot(time, torque_hist, label="actual torque")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Torque [N m]")
    ax2.set_title("Fast inner loop tracking slow outer-loop command")
    ax2.grid(True)
    ax2.legend()
    plt.show()


if __name__ == "__main__":
    main()
