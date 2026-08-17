import numpy as np
from scipy.optimize import minimize


class LinearMPC:
    """Small educational linear MPC with box-constrained inputs.

    It optimizes a finite input sequence and applies only the first control.
    This is intentionally compact, not production flight-control software.
    """

    def __init__(
        self,
        A: np.ndarray,
        B: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        horizon: int,
        u_min: float,
        u_max: float,
    ) -> None:
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.horizon = horizon
        self.u_min = u_min
        self.u_max = u_max

    def _cost(self, u_flat: np.ndarray, x0: np.ndarray, x_ref: np.ndarray) -> float:
        x = x0.copy()
        total = 0.0
        m = self.B.shape[1]

        for k in range(self.horizon):
            u = u_flat[k * m : (k + 1) * m]
            error = x - x_ref
            total += float(error.T @ self.Q @ error + u.T @ self.R @ u)
            x = self.A @ x + self.B @ u

        terminal_error = x - x_ref
        total += float(terminal_error.T @ self.Q @ terminal_error)
        return total

    def control(self, x: np.ndarray, x_ref: np.ndarray) -> np.ndarray:
        m = self.B.shape[1]
        guess = np.zeros(self.horizon * m)
        bounds = [(self.u_min, self.u_max)] * (self.horizon * m)
        result = minimize(
            self._cost,
            guess,
            args=(x, x_ref),
            bounds=bounds,
            method="SLSQP",
        )
        if not result.success:
            raise RuntimeError(f"MPC optimization failed: {result.message}")
        return result.x[:m]
