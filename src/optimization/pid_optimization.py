from dataclasses import dataclass

import numpy as np

from src.controller.pid import PIDGains
from src.optimization.cost_function import compute_cost
from src.simulation.pid_simulation import simulate_pid


@dataclass(frozen=True)
class PIDOptimizationConfig:
    omega_ref: float
    V_max: float
    simulation_time: float = 10.0
    dt: float = 0.001
    load_torque: float = 0.0
    penalty_cost: float = 1e6


def evaluate_pid_candidate(motor_params, gains, config):
    """Simulate one PID candidate and return cost plus response metrics."""
    try:
        result = simulate_pid(
            motor_params=motor_params,
            gains=gains,
            omega_ref=config.omega_ref,
            V_max=config.V_max,
            simulation_time=config.simulation_time,
            dt=config.dt,
            load_torque=config.load_torque,
        )
        record = compute_cost(result, omega_ref=config.omega_ref, V_max=config.V_max)
        unstable = not np.all(np.isfinite(result["omega"]))
        invalid = not np.isfinite(record["total"]) or unstable
        if invalid:
            record["total"] = config.penalty_cost
        return {
            **gains.as_dict(),
            **record,
            "valid": not invalid,
            "penalized": bool(invalid),
        }
    except Exception:
        return {
            **gains.as_dict(),
            "total": config.penalty_cost,
            "tracking": np.nan,
            "overshoot_cost": np.nan,
            "control": np.nan,
            "omega_final": np.nan,
            "omega_max": np.nan,
            "overshoot": np.nan,
            "overshoot_percent": np.nan,
            "settling_time": np.nan,
            "steady_state_error": np.nan,
            "steady_state_error_percent": np.nan,
            "voltage_max_abs": np.nan,
            "saturation_fraction": np.nan,
            "saturation_percent": np.nan,
            "valid": False,
            "penalized": True,
        }


def sample_random_gains(rng, bounds):
    """Sample PID gains uniformly from a bounds dictionary."""
    return PIDGains(
        K_p=float(rng.uniform(*bounds["K_p"])),
        K_i=float(rng.uniform(*bounds["K_i"])),
        K_d=float(rng.uniform(*bounds["K_d"])),
    )


def random_search_pid(motor_params, bounds, n_trials, config, seed=42):
    """Run reproducible Random Search over PID gains."""
    rng = np.random.default_rng(seed)
    records = []
    for trial in range(1, n_trials + 1):
        gains = sample_random_gains(rng, bounds)
        record = evaluate_pid_candidate(motor_params, gains, config)
        record["trial"] = trial
        records.append(record)
    best = min(records, key=lambda r: r["total"])
    return records, best
