import numpy as np


class DiscreteKalmanFilter:
    """Linear discrete Kalman filter.

    Model:
        x[k+1] = A x[k] + B u[k] + w
        y[k]   = C x[k] + v
    """

    def __init__(
        self,
        A: np.ndarray,
        B: np.ndarray,
        C: np.ndarray,
        Qn: np.ndarray,
        Rn: np.ndarray,
        x0: np.ndarray,
        P0: np.ndarray,
    ) -> None:
        self.A = A
        self.B = B
        self.C = C
        self.Qn = Qn
        self.Rn = Rn
        self.x = x0.astype(float).copy()
        self.P = P0.astype(float).copy()

    def predict(self, u: np.ndarray) -> np.ndarray:
        self.x = self.A @ self.x + self.B @ u
        self.P = self.A @ self.P @ self.A.T + self.Qn
        return self.x

    def correct(self, y: np.ndarray) -> np.ndarray:
        innovation = y - self.C @ self.x
        S = self.C @ self.P @ self.C.T + self.Rn
        K = self.P @ self.C.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.C) @ self.P
        return self.x

    def update(self, u: np.ndarray, y: np.ndarray) -> np.ndarray:
        self.predict(u)
        return self.correct(y)
