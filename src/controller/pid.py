

class PIDController:
    """
    Basic PID Controller

    u(t) =
        Kp * e(t)
        + Ki * integral(e)
        + Kd * de/dt
    """

    def __init__(
        self,
        kp,
        ki,
        kd,
        output_min=None,
        output_max=None
    ):

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.previous_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, reference, measurement, dt):

        # Tracking error
        error = reference - measurement

        # Integral
        self.integral += error * dt

        # Derivative
        derivative = (
            error - self.previous_error
        ) / dt

        # PID equation
        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        # Voltage saturation
        if self.output_min is not None:
            output = max(
                self.output_min,
                output
            )

        if self.output_max is not None:
            output = min(
                self.output_max,
                output
            )

        self.previous_error = error

        return output, error
