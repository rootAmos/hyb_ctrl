import numpy as np
import matplotlib.pyplot as plt

from hyb_ctrl.two_inertia import TwoInertiaDrivetrain


def main() -> None:
    dt = 0.0005
    time = np.arange(0.0, 2.0, dt)
    plant = TwoInertiaDrivetrain()

    omega_1 = np.zeros_like(time)
    omega_2 = np.zeros_like(time)
    phi = np.zeros_like(time)

    for k, t in enumerate(time):
        torque_cmd = 4.0 if t >= 0.1 else 0.0
        load = 1.0 if t >= 1.0 else 0.0
        omega_1[k], omega_2[k], phi[k] = plant.step(torque_cmd, load, dt)

    fig, ax = plt.subplots()
    ax.plot(time, omega_1, label="motor-side speed")
    ax.plot(time, omega_2, label="load-side speed")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [rad/s]")
    ax.set_title("Week 2: two-inertia torsional response")
    ax.grid(True)
    ax.legend()

    fig2, ax2 = plt.subplots()
    ax2.plot(time, phi)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Twist [rad]")
    ax2.set_title("Shaft torsional mode")
    ax2.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
