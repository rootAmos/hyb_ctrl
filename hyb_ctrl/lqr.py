import numpy as np
from scipy.linalg import solve_discrete_are


def dlqr(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Discrete-time LQR gain for x[k+1] = A x[k] + B u[k].

    Returns:
        K: state-feedback gain used as u = -K x
        P: Riccati solution / value-function matrix
    """
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    return K, P
