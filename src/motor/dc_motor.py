
from dataclasses import dataclass


@dataclass
class DCMotorParams:
    """
    DC Motor physical parameters
    """
    R: float       # Armature resistance [ohm]
    L: float       # Armature inductance [H]
    Jm: float      # Rotor inertia [kg*m^2]
    b: float       # Viscous friction [N*m*s/rad]
    Kt: float      # Torque constant [N*m/A]
    Ke: float      # Back-EMF constant [V*s/rad]


class DCMotor:
    """
    Armature-controlled DC Motor

    States
    ------
    i     : armature current [A]
    omega : angular velocity [rad/s]
    """

    def __init__(self, params: DCMotorParams):
        self.params = params

        self.current = 0.0
        self.omega = 0.0

    def reset(self):
        """
        Reset motor state.
        """
        self.current = 0.0
        self.omega = 0.0

    def derivatives(self, voltage, load_torque=0.0):
        """
        Calculate state derivatives.

        Electrical:
            di/dt =
            -(R/L)i
            -(Ke/L)omega
            +(1/L)V

        Mechanical:
            domega/dt =
            (Kt/Jm)i
            -(b/Jm)omega
            -(1/Jm)TL
        """

        p = self.params

        di_dt = (
            -(p.R / p.L) * self.current
            -(p.Ke / p.L) * self.omega
            +(1.0 / p.L) * voltage
        )

        domega_dt = (
            (p.Kt / p.Jm) * self.current
            -(p.b / p.Jm) * self.omega
            -(1.0 / p.Jm) * load_torque
        )

        return di_dt, domega_dt

    def step(self, voltage, dt, load_torque=0.0):
        """
        Advance motor state by one time step.

        v1 uses Euler integration.
        """

        di_dt, domega_dt = self.derivatives(
            voltage,
            load_torque
        )

        self.current += di_dt * dt
        self.omega += domega_dt * dt

        return self.current, self.omega
