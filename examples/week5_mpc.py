import numpy as np
import matplotlib.pyplot as plt

from hyb_ctrl.mpc import LinearMPC


def main() -> None:
    dt = 0.1
    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([[0.5 * dt**2], [dt]])

    controller = LinearMPC(
        A=A,
        B=B,
        Q=np.diag([20.0, 2.0]),
        R=np.array([[0.2]]),
        horizon=20,
        u_min=-1.0,
        u_max=1.0,
    )

    x = np.array([0.0, 0.0])
    x_ref = np.array([5.0, 0.0])
    time = np.arange(0.0, 8.0, dt)
    state_hist = np.zeros((len(time), 2))
    control_hist = np.zeros(len(time))

    for k, _ in enumerate(time):
        u = controller.control(x, x_ref)
        x = A @ x + B @ u
        state_hist[k] = x
        control_hist[k] = u.item()

    fig, ax = plt.subplots()
    ax.plot(time, state_hist[:, 0], label="state 1")
    ax.axhline(x_ref[0], linestyle="--", label="target")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("State")
    ax.set_title("Week 5: MPC planning toward a target")
    ax.grid(True)
    ax.legend()

    fig2, ax2 = plt.subplots()
    ax2.plot(time, control_hist)
    ax2.axhline(1.0, linestyle="--", label="upper input limit")
    ax2.axhline(-1.0, linestyle="--", label="lower input limit")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Control input")
    ax2.set_title("MPC respects actuator constraints")
    ax2.grid(True)
    ax2.legend()
    plt.show()


if __name__ == "__main__":
    main()
