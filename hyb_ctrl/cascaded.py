from dataclasses import dataclass

from hyb_ctrl.controllers import PIController


@dataclass
class CascadedSpeedTorqueController:
    """Week 4 cascaded controller.

    Outer loop: speed error -> torque reference.
    Inner loop: measured torque/current proxy -> actuator command.

    The inner loop should be tuned substantially faster than the outer loop.
    """

    speed_loop: PIController
    torque_loop: PIController

    def update(
        self,
        speed_ref: float,
        speed_meas: float,
        torque_meas: float,
        dt_outer: float,
        dt_inner: float,
    ) -> tuple[float, float]:
        torque_ref = self.speed_loop.update(speed_ref, speed_meas, dt_outer)
        actuator_cmd = self.torque_loop.update(torque_ref, torque_meas, dt_inner)
        return torque_ref, actuator_cmd
