from dataclasses import dataclass


@dataclass(frozen=True)
class PIDGains:
    """PID gain set."""

    K_p: float
    K_i: float
    K_d: float

    @property
    def kp(self):
        return self.K_p

    @property
    def ki(self):
        return self.K_i

    @property
    def kd(self):
        return self.K_d

    def as_dict(self):
        return {"K_p": self.K_p, "K_i": self.K_i, "K_d": self.K_d}


class PIDController:
    """Conventional PID controller with voltage saturation."""

    def __init__(
        self,
        K_p=None,
        K_i=None,
        K_d=None,
        kp=None,
        ki=None,
        kd=None,
        output_min=None,
        output_max=None,
        anti_windup=True,
    ):
        self.K_p = float(K_p if K_p is not None else kp)
        self.K_i = float(K_i if K_i is not None else ki)
        self.K_d = float(K_d if K_d is not None else kd)
        self.output_min = output_min
        self.output_max = output_max
        self.anti_windup = anti_windup
        self.integral = 0.0
        self.previous_error = None

    @classmethod
    def from_gains(cls, gains: PIDGains, **kwargs):
        return cls(gains.K_p, gains.K_i, gains.K_d, **kwargs)

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def _saturate(self, value):
        if self.output_min is not None:
            value = max(self.output_min, value)
        if self.output_max is not None:
            value = min(self.output_max, value)
        return value

    def compute(self, reference, measurement, dt):
        if dt <= 0:
            raise ValueError("dt must be positive.")

        error = reference - measurement
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        candidate_integral = self.integral + error * dt

        raw_output = (
            self.K_p * error
            + self.K_i * candidate_integral
            + self.K_d * derivative
        )
        output = self._saturate(raw_output)

        saturated_high = self.output_max is not None and raw_output > self.output_max
        saturated_low = self.output_min is not None and raw_output < self.output_min
        pushing_deeper = (saturated_high and error > 0) or (saturated_low and error < 0)

        if not self.anti_windup or not pushing_deeper:
            self.integral = candidate_integral

        self.previous_error = error

        return output, error
