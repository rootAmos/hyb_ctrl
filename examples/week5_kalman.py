import numpy as np
import matplotlib.pyplot as plt

from hyb_ctrl.kalman import DiscreteKalmanFilter


def main() -> None:
    rng = np.random.default_rng(4)

    dt = 0.02
    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([[0.5 * dt**2], [dt]])
    C = np.array([[1.0, 0.0]])

    kf = DiscreteKalmanFilter(
        A=A,
        B=B,
        C=C,
        Qn=np.diag([1e-5, 1e-4]),
        Rn=np.array([[0.05**2]]),
        x0=np.zeros(2),
        P0=np.eye(2),
    )

    time = np.arange(0.0, 6.0, dt)
    x_true = np.zeros(2)
    true_hist = np.zeros((len(time), 2))
    est_hist = np.zeros((len(time), 2))
    meas_hist = np.zeros(len(time))

    for k, t in enumerate(time):
        u = np.array([1.0 if t < 2.0 else 0.0])
        x_true = A @ x_true + B @ u
        y = C @ x_true + rng.normal(0.0, 0.05, size=1)
        x_est = kf.update(u, y)

        true_hist[k] = x_true
        est_hist[k] = x_est
        meas_hist[k] = y.item()

    fig, ax = plt.subplots()
    ax.plot(time, true_hist[:, 0], label="true position/state")
    ax.plot(time, meas_hist, alpha=0.45, label="noisy measurement")
    ax.plot(time, est_hist[:, 0], label="Kalman estimate")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("State 1")
    ax.set_title("Week 5: model prediction + sensor correction")
    ax.grid(True)
    ax.legend()

    fig2, ax2 = plt.subplots()
    ax2.plot(time, true_hist[:, 1], label="true unmeasured state")
    ax2.plot(time, est_hist[:, 1], label="estimated unmeasured state")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("State 2")
    ax2.set_title("Kalman filter reconstructing an unmeasured state")
    ax2.grid(True)
    ax2.legend()
    plt.show()


if __name__ == "__main__":
    main()
