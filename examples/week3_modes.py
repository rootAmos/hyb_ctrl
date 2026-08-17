import numpy as np

from hyb_ctrl.state_space import (
    controllability_matrix,
    modes,
    observability_matrix,
    two_inertia_matrices,
)


def main() -> None:
    A, B = two_inertia_matrices(J1=0.05, J2=0.08, k=8.0, c=0.15)
    C = np.array([[1.0, 0.0, 0.0]])

    eigenvalues, eigenvectors = modes(A)
    ctrb = controllability_matrix(A, B)
    obsv = observability_matrix(A, C)

    print("A =\n", A)
    print("B =\n", B)
    print("Eigenvalues =", eigenvalues)
    print("Controllability rank =", np.linalg.matrix_rank(ctrb))
    print("Observability rank =", np.linalg.matrix_rank(obsv))
    print("Eigenvectors =\n", eigenvectors)


if __name__ == "__main__":
    main()
