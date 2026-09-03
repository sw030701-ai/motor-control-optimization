from dataclasses import dataclass

import numpy as np

from src.controller.pid import PIDGains
from src.optimization.cost_function import compute_cost
from src.simulation.pid_simulation import simulate_pid


@dataclass(frozen=True)
class BaselineTuningConfig:
    omega_ref: float
    V_max: float
    simulation_time: float = 2.0
    dt: float = 0.0005
    settling_target: float = 0.5


def evaluate_gains(motor_params, gains, config):
    result = simulate_pid(
        motor_params=motor_params,
        gains=gains,
        omega_ref=config.omega_ref,
        V_max=config.V_max,
        simulation_time=config.simulation_time,
        dt=config.dt,
    )
    record = compute_cost(result, omega_ref=config.omega_ref, V_max=config.V_max)
    return {**gains.as_dict(), **record}


def scan_gain_candidates(motor_params, candidates, config):
    records = []
    for gains in candidates:
        records.append(evaluate_gains(motor_params, gains, config))
    return records


def choose_first(records, predicate):
    for record in records:
        if predicate(record):
            return record
    raise ValueError("No candidate satisfied the tuning rule.")


def sequential_baseline_tuning(motor_params, config):
    """Run a reproducible manual-style sequential PID tuning scan."""
    kp_candidates = [0.02, 0.04, 0.06, 0.08, 0.10, 0.15, 0.20, 0.30, 0.50, 0.80]
    kp_records = scan_gain_candidates(
        motor_params,
        [PIDGains(K_p=kp, K_i=0.0, K_d=0.0) for kp in kp_candidates],
        config,
    )
    kp_selected = choose_first(
        kp_records,
        lambda r: (
            r["omega_final"] >= 0.60 * config.omega_ref
            and r["overshoot"] <= 0.05
            and r["saturation_fraction"] < 0.01
        ),
    )

    ki_candidates = [0.05, 0.10, 0.20, 0.40, 0.80, 1.20, 2.00]
    ki_records = scan_gain_candidates(
        motor_params,
        [PIDGains(K_p=kp_selected["K_p"], K_i=ki, K_d=0.0) for ki in ki_candidates],
        config,
    )
    ki_selected = choose_first(
        ki_records,
        lambda r: (
            r["steady_state_error"] <= 0.02
            and r["settling_time"] <= config.settling_target
            and r["overshoot"] <= 0.10
            and r["saturation_fraction"] < 0.05
        ),
    )

    kd_candidates = [0.0, 0.0001, 0.0002, 0.0005, 0.0010, 0.0020]
    kd_records = scan_gain_candidates(
        motor_params,
        [PIDGains(K_p=ki_selected["K_p"], K_i=ki_selected["K_i"], K_d=kd) for kd in kd_candidates],
        config,
    )
    kd_selected = min(
        [
            r for r in kd_records
            if (
                r["overshoot"] <= 0.10
                and r["steady_state_error"] <= 0.02
                and r["saturation_fraction"] < 0.05
                and np.isfinite(r["settling_time"])
            )
        ],
        key=lambda r: (r["settling_time"], r["overshoot"], r["voltage_max_abs"]),
    )

    final_gains = PIDGains(
        K_p=kd_selected["K_p"],
        K_i=kd_selected["K_i"],
        K_d=kd_selected["K_d"],
    )

    return {
        "Kp_scan": kp_records,
        "Ki_scan": ki_records,
        "Kd_scan": kd_records,
        "selected": kd_selected,
        "final_gains": final_gains,
    }
