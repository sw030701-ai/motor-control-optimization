import math

from src.motor.dc_motor import nominal_dc_motor_params
from src.optimization.cost_function import accepted_baseline, compute_cost
from src.simulation.baseline_tuning import BaselineTuningConfig, sequential_baseline_tuning
from src.simulation.pid_simulation import (
    reference_from_reachable_speed,
    simulate_open_loop,
    simulate_pid,
)


def test_open_loop_matches_no_load_steady_state_speed():
    params = nominal_dc_motor_params()
    voltage = 12.0
    result = simulate_open_loop(params, voltage=voltage, simulation_time=10.0, dt=0.001)
    expected = params.no_load_steady_state_speed(voltage)

    assert math.isclose(result["omega"][-1], expected, rel_tol=2e-3)


def test_sequential_baseline_tuning_meets_v1_acceptance_criteria():
    params = nominal_dc_motor_params()
    V_max = 12.0
    omega_ref = round(reference_from_reachable_speed(params, V_max, fraction=0.5), 2)
    config = BaselineTuningConfig(omega_ref=omega_ref, V_max=V_max)

    tuning = sequential_baseline_tuning(params, config)
    result = simulate_pid(
        params,
        gains=tuning["final_gains"],
        omega_ref=omega_ref,
        V_max=V_max,
        simulation_time=config.simulation_time,
        dt=config.dt,
    )
    record = compute_cost(result, omega_ref=omega_ref, V_max=V_max)

    assert tuning["final_gains"].K_p == 0.8
    assert tuning["final_gains"].K_i == 2.0
    assert tuning["final_gains"].K_d == 0.002
    assert accepted_baseline(record)
