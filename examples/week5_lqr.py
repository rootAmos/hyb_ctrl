import numpy as np
import matplotlib.pyplot as plt

from hyb_ctrl.discrete import forward_euler
from hyb_ctrl.lqr import dlqr


def main() -> None:
    # Simple two-state speed + integral-like state model for LQR practice.
    A = np.array([[-0.4, 0.0], [1.0, 0.0]])
    B = np.array([[1.0], [0.0]])
    dt = 0.02
    Ad, Bd = forward_euler(A, B, dt)

    Q = np.diag([10.0, 1.0])
    R = np.array([[0.5]])
    K, _ = dlqr(Ad, Bd, Q, R)

    x = np.array([5.0, 0.0])
    time = np.arange(0.0, 5.0, dt)
    history = np.zeros((len(time), 2))
    control = np.zeros(len(time))

    for k, _ in enumerate(time):
        u = -K @ x
        x = Ad @ x + Bd @ u
        history[k] = x
        control[k] = u.item()

    print("LQR gain K =", K)
    print("Closed-loop eigenvalues =", np.linalg.eigvals(Ad - Bd @ K))

    fig, ax = plt.subplots()
    ax.plot(time, history[:, 0], label="state 1")
    ax.plot(time, history[:, 1], label="state 2")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("State")
    ax.set_title("Week 5: LQR state feedback")
    ax.grid(True)
    ax.legend()

    fig2, ax2 = plt.subplots()
    ax2.plot(time, control)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Control input")
    ax2.set_title("LQR control effort")
    ax2.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
