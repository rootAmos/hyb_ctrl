import numpy as np
import matplotlib.pyplot as plt

from hyb_ctrl.controllers import PIController
from hyb_ctrl.motor import SingleInertiaMotor


def main() -> None:
    dt = 0.001
    t_final = 4.0
    time = np.arange(0.0, t_final, dt)

    motor = SingleInertiaMotor(J=0.05, b=0.02)
    controller = PIController(kp=0.35, ki=2.0, u_min=-15.0, u_max=15.0)

    omega_ref = 100.0  # rad/s
    omega = np.zeros_like(time)
    torque_cmd = np.zeros_like(time)
    torque_load = np.zeros_like(time)

    for k, t in enumerate(time):
        # At t = 2 s, suddenly apply a resisting load torque.
        load = 2.0 if t >= 2.0 else 0.0
        command = controller.update(omega_ref, motor.omega, dt)

        torque_load[k] = load
        torque_cmd[k] = command
        omega[k] = motor.step(command, load, dt)

    fig, ax = plt.subplots()
    ax.plot(time, omega, label="shaft speed")
    ax.axhline(omega_ref, linestyle="--", label="reference")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Shaft speed [rad/s]")
    ax.set_title("Week 1: PI speed control with load disturbance")
    ax.grid(True)
    ax.legend()

    fig2, ax2 = plt.subplots()
    ax2.plot(time, torque_cmd, label="motor torque command")
    ax2.plot(time, torque_load, label="load torque")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Torque [N m]")
    ax2.set_title("Controller effort")
    ax2.grid(True)
    ax2.legend()

    plt.show()


if __name__ == "__main__":
    main()
