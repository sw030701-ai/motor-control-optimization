import warnings
from dataclasses import dataclass, field

import numpy as np

from src.controller.pid import PIDGains
from src.optimization.cost_function import (
    V1_CONSTRAINTS,
    V1ConstraintPolicy,
    compute_cost,
    is_feasible_v1,
    v1_constraint_checks,
)
from src.simulation.pid_simulation import simulate_pid


@dataclass(frozen=True)
class PIDOptimizationConfig:
    omega_ref: float
    V_max: float
    simulation_time: float = 10.0
    dt: float = 0.001
    load_torque: float = 0.0
    penalty_cost: float = 1e6
    feasibility_policy: V1ConstraintPolicy = field(
        default_factory=lambda: V1_CONSTRAINTS
    )


def _constraint_columns(record, policy):
    checks = v1_constraint_checks(record, policy)
    return {f"constraint_{name}": bool(value) for name, value in checks.items()}


def evaluate_pid_candidate(motor_params, gains, config, enforce_constraints=False):
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
            objective = config.penalty_cost
            record["total"] = config.penalty_cost
        else:
            objective = record["total"]
        record["valid"] = not invalid
        feasible = is_feasible_v1(record, config.feasibility_policy)
        if enforce_constraints and not feasible:
            objective = config.penalty_cost
        return {
            **gains.as_dict(),
            **record,
            **_constraint_columns(record, config.feasibility_policy),
            "valid": not invalid,
            "feasible": bool(feasible),
            "objective": float(objective),
            "penalized": bool(invalid or (enforce_constraints and not feasible)),
        }
    except Exception:
        record = {
            **gains.as_dict(),
            "total": config.penalty_cost,
            "tracking": np.nan,
            "overshoot_cost": np.nan,
            "control": np.nan,
            "omega_final": np.nan,
            "omega_max": np.nan,
            "omega_min": np.nan,
            "omega_abs_max": np.nan,
            "overshoot": np.nan,
            "overshoot_percent": np.nan,
            "settling_time": np.nan,
            "steady_state_error": np.nan,
            "steady_state_error_percent": np.nan,
            "voltage_max_abs": np.nan,
            "saturation_fraction": np.nan,
            "saturation_percent": np.nan,
            "valid": False,
            "feasible": False,
            "objective": config.penalty_cost,
            "penalized": True,
        }
        return {
            **record,
            **_constraint_columns(record, config.feasibility_policy),
        }


def select_best_feasible(records):
    """Select the feasible candidate with the smallest unpenalized total cost."""
    feasible_records = [record for record in records if record.get("feasible", False)]
    if not feasible_records:
        raise ValueError("No feasible PID candidate found under the v1 constraints.")
    return min(feasible_records, key=lambda record: record["total"])


def sample_random_gains(rng, bounds):
    """Sample PID gains uniformly from a bounds dictionary."""
    return PIDGains(
        K_p=float(rng.uniform(*bounds["K_p"])),
        K_i=float(rng.uniform(*bounds["K_i"])),
        K_d=float(rng.uniform(*bounds["K_d"])),
    )


def random_search_pid(
    motor_params, bounds, n_trials, config, seed=42, constrained=False
):
    """Run reproducible Random Search over PID gains."""
    rng = np.random.default_rng(seed)
    records = []
    for trial in range(1, n_trials + 1):
        gains = sample_random_gains(rng, bounds)
        record = evaluate_pid_candidate(
            motor_params,
            gains,
            config,
            enforce_constraints=constrained,
        )
        record["trial"] = trial
        records.append(record)
    best = (
        select_best_feasible(records)
        if constrained
        else min(records, key=lambda r: r["total"])
    )
    return records, best


def constrained_random_search_pid(motor_params, bounds, n_trials, config, seed=42):
    """Run Random Search and return the lowest-cost feasible PID candidate."""
    return random_search_pid(
        motor_params=motor_params,
        bounds=bounds,
        n_trials=n_trials,
        config=config,
        seed=seed,
        constrained=True,
    )


def _bounds_arrays(bounds):
    names = ["K_p", "K_i", "K_d"]
    lows = np.array([bounds[name][0] for name in names], dtype=float)
    highs = np.array([bounds[name][1] for name in names], dtype=float)
    return names, lows, highs


def _scale(x, lows, highs):
    return (np.asarray(x, dtype=float) - lows) / (highs - lows)


def _unscale(z, lows, highs):
    return lows + np.asarray(z, dtype=float) * (highs - lows)


def constrained_bayesian_optimization_pid(
    motor_params,
    bounds,
    n_trials,
    config,
    seed=42,
    n_initial=8,
    acquisition_pool_size=2000,
):
    """Run Bayesian Optimization with infeasible candidates receiving a hard penalty."""
    try:
        from scipy.stats import norm
        from sklearn.gaussian_process import GaussianProcessRegressor
        from sklearn.gaussian_process.kernels import ConstantKernel, Matern, WhiteKernel
    except ImportError as exc:
        raise ImportError(
            "scipy and scikit-learn are required for constrained Bayesian Optimization."
        ) from exc

    rng = np.random.default_rng(seed)
    names, lows, highs = _bounds_arrays(bounds)
    records = []
    x_history = []
    n_initial = min(n_initial, n_trials)

    def evaluate_array(x, trial):
        gains = PIDGains(K_p=float(x[0]), K_i=float(x[1]), K_d=float(x[2]))
        record = evaluate_pid_candidate(
            motor_params,
            gains,
            config,
            enforce_constraints=True,
        )
        record["trial"] = trial
        return record

    for trial in range(1, n_initial + 1):
        x = _unscale(rng.random(len(names)), lows, highs)
        x_history.append(x)
        records.append(evaluate_array(x, trial))

    for trial in range(n_initial + 1, n_trials + 1):
        x_train = _scale(np.vstack(x_history), lows, highs)
        y_train = np.array([record["objective"] for record in records], dtype=float)

        kernel = ConstantKernel(1.0, constant_value_bounds=(1e-3, 1e3)) * Matern(
            length_scale=np.ones(len(names)), nu=2.5
        ) + WhiteKernel(noise_level=1e-8, noise_level_bounds=(1e-10, 1e-3))
        model = GaussianProcessRegressor(
            kernel=kernel,
            normalize_y=True,
            random_state=seed,
            n_restarts_optimizer=2,
        )

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            model.fit(x_train, y_train)

        candidate_pool = rng.random((acquisition_pool_size, len(names)))
        mu, sigma = model.predict(candidate_pool, return_std=True)
        sigma = np.maximum(sigma, 1e-12)
        best_observed = np.min(y_train)
        improvement = best_observed - mu
        z = improvement / sigma
        expected_improvement = improvement * norm.cdf(z) + sigma * norm.pdf(z)

        x_next = _unscale(
            candidate_pool[int(np.argmax(expected_improvement))], lows, highs
        )
        x_history.append(x_next)
        records.append(evaluate_array(x_next, trial))

    best = select_best_feasible(records)
    return records, best
