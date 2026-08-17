import numpy as np


def forward_euler(A: np.ndarray, B: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Simple continuous-to-discrete approximation for learning and comparison."""
    n = A.shape[0]
    Ad = np.eye(n) + A * dt
    Bd = B * dt
    return Ad, Bd
