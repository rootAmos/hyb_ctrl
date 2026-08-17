import numpy as np

from hyb_ctrl.controllers import PIController
from hyb_ctrl.discrete import forward_euler
from hyb_ctrl.motor import SingleInertiaMotor
from hyb_ctrl.state_space import controllability_matrix, two_inertia_matrices


def test_motor_accelerates_under_positive_torque() -> None:
    motor = SingleInertiaMotor(J=0.05, b=0.02)
    motor.step(torque_cmd=1.0, torque_load=0.0, dt=0.001)
    assert motor.omega > 0.0


def test_pi_saturates() -> None:
    controller = PIController(kp=10.0, ki=0.0, u_min=-2.0, u_max=2.0)
    assert controller.update(reference=10.0, measurement=0.0, dt=0.01) == 2.0


def test_two_inertia_model_is_controllable() -> None:
    A, B = two_inertia_matrices(J1=0.05, J2=0.08, k=8.0, c=0.15)
    assert np.linalg.matrix_rank(controllability_matrix(A, B)) == A.shape[0]


def test_forward_euler_dimensions() -> None:
    A = np.array([[-1.0]])
    B = np.array([[2.0]])
    Ad, Bd = forward_euler(A, B, dt=0.1)
    np.testing.assert_allclose(Ad, [[0.9]])
    np.testing.assert_allclose(Bd, [[0.2]])
