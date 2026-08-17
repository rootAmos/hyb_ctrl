import numpy as np


def two_inertia_matrices(J1: float, J2: float, k: float, c: float) -> tuple[np.ndarray, np.ndarray]:
    """Return continuous-time state-space matrices for x=[omega1, omega2, phi]."""
    A = np.array(
        [
            [-c / J1, c / J1, -k / J1],
            [c / J2, -c / J2, k / J2],
            [1.0, -1.0, 0.0],
        ]
    )
    B = np.array([[1.0 / J1], [0.0], [0.0]])
    return A, B


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    n = A.shape[0]
    blocks = [B]
    term = B.copy()
    for _ in range(1, n):
        term = A @ term
        blocks.append(term)
    return np.hstack(blocks)


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    n = A.shape[0]
    blocks = [C]
    term = C.copy()
    for _ in range(1, n):
        term = term @ A
        blocks.append(term)
    return np.vstack(blocks)


def modes(A: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Eigenvalues and right eigenvectors of A."""
    return np.linalg.eig(A)
