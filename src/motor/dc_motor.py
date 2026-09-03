from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DCMotorParams:
    """Armature-controlled DC motor physical parameters."""

    R: float
    L: float
    J_m: float
    b: float
    K_t: float
    K_e: float

    @property
    def Jm(self):
        """Backward-compatible alias for older notebooks."""
        return self.J_m

    @property
    def Kt(self):
        """Backward-compatible alias for older notebooks."""
        return self.K_t

    @property
    def Ke(self):
        """Backward-compatible alias for older notebooks."""
        return self.K_e

    def no_load_steady_state_speed(self, voltage):
        """Return no-load steady-state speed for a constant voltage."""
        return voltage / (self.K_e + (self.R * self.b / self.K_t))


def nominal_dc_motor_params():
    """Return the v1 nominal DC motor parameter set used by baseline experiments."""
    return DCMotorParams(
        R=2.0,
        L=0.0005,
        J_m=1.0e-4,
        b=1.0e-5,
        K_t=0.05,
        K_e=0.05,
    )


class DCMotor:
    """
    Armature-controlled DC motor model.

    State:
        i      armature current [A]
        omega  angular velocity [rad/s]
    """

    def __init__(self, params: DCMotorParams, current=0.0, omega=0.0):
        self.params = params
        self.current = float(current)
        self.omega = float(omega)

    def reset(self, current=0.0, omega=0.0):
        self.current = float(current)
        self.omega = float(omega)

    @property
    def state(self):
        return np.array([self.current, self.omega], dtype=float)

    def derivatives(self, voltage, load_torque=0.0, state=None):
        """Calculate [di/dt, domega/dt]."""
        p = self.params
        current, omega = self.state if state is None else state

        di_dt = (
            -(p.R / p.L) * current
            - (p.K_e / p.L) * omega
            + (1.0 / p.L) * voltage
        )

        domega_dt = (
            (p.K_t / p.J_m) * current
            - (p.b / p.J_m) * omega
            - (1.0 / p.J_m) * load_torque
        )

        return np.array([di_dt, domega_dt], dtype=float)

    def step(self, voltage, dt, load_torque=0.0, method="rk4"):
        """Advance motor state by one time step."""
        if dt <= 0:
            raise ValueError("dt must be positive.")

        state = self.state

        if method == "euler":
            next_state = state + dt * self.derivatives(voltage, load_torque, state)
        elif method == "rk4":
            k1 = self.derivatives(voltage, load_torque, state)
            k2 = self.derivatives(voltage, load_torque, state + 0.5 * dt * k1)
            k3 = self.derivatives(voltage, load_torque, state + 0.5 * dt * k2)
            k4 = self.derivatives(voltage, load_torque, state + dt * k3)
            next_state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        else:
            raise ValueError("method must be 'rk4' or 'euler'.")

        self.current = float(next_state[0])
        self.omega = float(next_state[1])

        return self.current, self.omega
