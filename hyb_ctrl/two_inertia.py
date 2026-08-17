from dataclasses import dataclass


@dataclass
class TwoInertiaDrivetrain:
    """Two inertias coupled by a torsional spring and damper.

    States:
        omega_1 : motor-side speed
        omega_2 : load-side speed
        phi     : relative shaft twist
    """

    J1: float = 0.05
    J2: float = 0.08
    k: float = 8.0
    c: float = 0.15
    omega_1: float = 0.0
    omega_2: float = 0.0
    phi: float = 0.0

    def derivatives(self, torque_cmd: float, torque_load: float = 0.0) -> tuple[float, float, float]:
        twist_torque = self.k * self.phi
        damping_torque = self.c * (self.omega_1 - self.omega_2)

        domega_1 = (torque_cmd - twist_torque - damping_torque) / self.J1
        domega_2 = (twist_torque + damping_torque - torque_load) / self.J2
        dphi = self.omega_1 - self.omega_2
        return domega_1, domega_2, dphi

    def step(self, torque_cmd: float, torque_load: float, dt: float) -> tuple[float, float, float]:
        domega_1, domega_2, dphi = self.derivatives(torque_cmd, torque_load)
        self.omega_1 += domega_1 * dt
        self.omega_2 += domega_2 * dt
        self.phi += dphi * dt
        return self.omega_1, self.omega_2, self.phi
