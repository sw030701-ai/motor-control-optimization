import math

import numpy as np
import pytest

from src.motor.dc_motor import nominal_dc_motor_params
from src.optimization.cost_function import accepted_baseline, compute_cost
from src.optimization.pid_optimization import (
    PIDOptimizationConfig,
    constraint_violations_v1,
    feasibility_aware_bayesian_optimization_pid,
    random_search_pid,
)
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


def test_random_search_pid_returns_best_candidate_from_history():
    params = nominal_dc_motor_params()
    config = PIDOptimizationConfig(
        omega_ref=12.6,
        V_max=12.0,
        simulation_time=0.2,
        dt=0.001,
    )
    bounds = {
        "K_p": (0.02, 0.8),
        "K_i": (0.05, 2.0),
        "K_d": (0.0, 0.002),
    }

    records, best = random_search_pid(
        motor_params=params,
        bounds=bounds,
        n_trials=5,
        config=config,
        seed=42,
    )

    assert len(records) == 5
    assert best in records
    assert best["total"] == min(record["total"] for record in records)


def test_constraint_violations_v1_use_nonpositive_feasible_convention():
    record = {
        "total": 0.1,
        "overshoot": 0.08,
        "steady_state_error": 0.01,
        "settling_time": 1.5,
        "saturation_fraction": 0.02,
        "valid": True,
    }

    violations = constraint_violations_v1(record)

    assert violations["overshoot"] <= 0.0
    assert violations["steady_state_error"] <= 0.0
    assert violations["settling_time"] <= 0.0
    assert violations["saturation"] <= 0.0


def test_constraint_violations_v1_clip_invalid_records_to_finite_violations():
    violations = constraint_violations_v1({"valid": False, "total": np.nan})

    assert all(np.isfinite(value) for value in violations.values())
    assert all(value > 0.0 for value in violations.values())


def test_feasibility_aware_bo_keeps_penalty_out_of_objective_surrogate():
    pytest.importorskip("scipy")
    pytest.importorskip("sklearn")

    params = nominal_dc_motor_params()
    config = PIDOptimizationConfig(
        omega_ref=12.6,
        V_max=12.0,
        simulation_time=10.0,
        dt=0.001,
    )
    bounds = {
        "K_p": (0.20, 1.50),
        "K_i": (0.50, 4.00),
        "K_d": (0.00, 0.01),
    }

    records, best = feasibility_aware_bayesian_optimization_pid(
        motor_params=params,
        bounds=bounds,
        n_trials=8,
        config=config,
        seed=42,
        n_initial=8,
        acquisition_pool_size=32,
    )

    assert len(records) == 8
    assert best["feasible"]
    assert all(not record["used_penalty_in_objective_model"] for record in records)
    assert all(
        record["objective_model_target"] != config.penalty_cost for record in records
    )
