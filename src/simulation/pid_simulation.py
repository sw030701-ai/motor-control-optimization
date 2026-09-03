
import numpy as np

from src.motor.dc_motor import DCMotor
from src.controller.pid import PIDController


def simulate_pid(
    motor_params,
    kp,
    ki,
    kd,
    omega_ref,
    vmax,
    simulation_time=5.0,
    dt=0.001,
    load_torque=0.0
):
    """
    Closed-loop DC motor PID simulation.
    """

    motor = DCMotor(motor_params)

    controller = PIDController(
        kp=kp,
        ki=ki,
        kd=kd,
        output_min=-vmax,
        output_max=vmax
    )

    time = np.arange(
        0,
        simulation_time + dt,
        dt
    )

    omega_history = np.zeros_like(time)
    current_history = np.zeros_like(time)
    voltage_history = np.zeros_like(time)
    error_history = np.zeros_like(time)

    for k, t in enumerate(time):

        voltage, error = controller.compute(
            reference=omega_ref,
            measurement=motor.omega,
            dt=dt
        )

        current, omega = motor.step(
            voltage=voltage,
            dt=dt,
            load_torque=load_torque
        )

        omega_history[k] = omega
        current_history[k] = current
        voltage_history[k] = voltage
        error_history[k] = error

    return {
        "time": time,
        "omega": omega_history,
        "current": current_history,
        "voltage": voltage_history,
        "error": error_history
    }
