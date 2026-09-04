import numpy as np

from src.controller.pid import PIDController, PIDGains
from src.motor.dc_motor import DCMotor


def _time_vector(simulation_time, dt):
    if simulation_time <= 0:
        raise ValueError("simulation_time must be positive.")
    if dt <= 0:
        raise ValueError("dt must be positive.")
    return np.arange(0.0, simulation_time + 0.5 * dt, dt)


def simulate_open_loop(
    motor_params,
    voltage,
    simulation_time=10.0,
    dt=0.001,
    load_torque=0.0,
    method="rk4",
):
    """Simulate DC motor response to a constant input voltage."""
    motor = DCMotor(motor_params)
    time = _time_vector(simulation_time, dt)

    omega_history = np.zeros_like(time)
    current_history = np.zeros_like(time)
    voltage_history = np.full_like(time, voltage, dtype=float)

    for k, _ in enumerate(time):
        current, omega = motor.step(
            voltage=voltage,
            dt=dt,
            load_torque=load_torque,
            method=method,
        )
        current_history[k] = current
        omega_history[k] = omega

    return {
        "time": time,
        "omega": omega_history,
        "current": current_history,
        "voltage": voltage_history,
    }


def simulate_pid(
    motor_params,
    K_p=None,
    K_i=None,
    K_d=None,
    kp=None,
    ki=None,
    kd=None,
    gains=None,
    omega_ref=100.0,
    V_max=12.0,
    vmax=None,
    simulation_time=10.0,
    dt=0.001,
    load_torque=0.0,
    method="rk4",
):
    """Closed-loop DC motor PID speed-control simulation."""
    if gains is None:
        gains = PIDGains(
            K_p=K_p if K_p is not None else kp,
            K_i=K_i if K_i is not None else ki,
            K_d=K_d if K_d is not None else kd,
        )

    voltage_limit = float(V_max if vmax is None else vmax)
    motor = DCMotor(motor_params)
    controller = PIDController.from_gains(
        gains,
        output_min=-voltage_limit,
        output_max=voltage_limit,
    )

    time = _time_vector(simulation_time, dt)
    omega_history = np.zeros_like(time)
    current_history = np.zeros_like(time)
    voltage_history = np.zeros_like(time)
    error_history = np.zeros_like(time)

    for k, _ in enumerate(time):
        voltage, error = controller.compute(
            reference=omega_ref,
            measurement=motor.omega,
            dt=dt,
        )

        current, omega = motor.step(
            voltage=voltage,
            dt=dt,
            load_torque=load_torque,
            method=method,
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
        "error": error_history,
        "gains": gains.as_dict(),
        "omega_ref": omega_ref,
        "V_max": voltage_limit,
    }


def reference_from_reachable_speed(motor_params, V_max, fraction=0.5):
    """Select a reachable reference speed from no-load maximum speed."""
    omega_ss_max = motor_params.no_load_steady_state_speed(V_max)
    return fraction * omega_ss_max
