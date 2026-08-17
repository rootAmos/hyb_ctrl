from dataclasses import dataclass


@dataclass
class PIController:
    """Discrete PI controller with simple output saturation."""

    kp: float
    ki: float
    u_min: float = float("-inf")
    u_max: float = float("inf")
    integral: float = 0.0

    def update(self, reference: float, measurement: float, dt: float) -> float:
        error = reference - measurement
        candidate_integral = self.integral + error * dt
        u_unsat = self.kp * error + self.ki * candidate_integral
        u = min(max(u_unsat, self.u_min), self.u_max)

        # Conditional integration: stop integrating farther into saturation.
        if u == u_unsat or (u == self.u_max and error < 0.0) or (u == self.u_min and error > 0.0):
            self.integral = candidate_integral

        return u
