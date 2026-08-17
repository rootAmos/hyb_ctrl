from dataclasses import dataclass


@dataclass
class SingleInertiaMotor:
    """Single-inertia shaft model.

    Dynamics:
        J * domega/dt = torque_cmd - torque_load - b * omega
    """

    J: float = 0.05
    b: float = 0.02
    omega: float = 0.0

    def derivative(self, torque_cmd: float, torque_load: float = 0.0) -> float:
        return (torque_cmd - torque_load - self.b * self.omega) / self.J

    def step(self, torque_cmd: float, torque_load: float, dt: float) -> float:
        self.omega += self.derivative(torque_cmd, torque_load) * dt
        return self.omega
