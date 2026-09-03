import numpy as np


def _integrate(y, x):
    if hasattr(np, "trapezoid"):
        return np.trapezoid(y, x)
    return np.trapz(y, x)


def settling_time(time, omega, omega_ref, tolerance=0.02):
    """Return first time after which response stays within tolerance band."""
    band = tolerance * abs(omega_ref)
    if band == 0:
        return np.inf

    error = np.abs(omega_ref - omega)
    for idx, _ in enumerate(time):
        if np.all(error[idx:] <= band):
            return float(time[idx])
    return np.inf


def compute_response_metrics(result, omega_ref, V_max, tolerance=0.02):
    """Compute response metrics used by baseline and optimization experiments."""
    time = np.asarray(result["time"], dtype=float)
    omega = np.asarray(result["omega"], dtype=float)
    voltage = np.asarray(result["voltage"], dtype=float)

    if omega_ref == 0:
        raise ValueError("omega_ref must be nonzero for normalized metrics.")
    if V_max <= 0:
        raise ValueError("V_max must be positive.")

    final_window_start = int(0.9 * len(time))
    final_window = omega[final_window_start:]

    omega_max = float(np.max(omega))
    overshoot = max(0.0, (omega_max - omega_ref) / omega_ref)
    steady_state_error = abs(omega_ref - float(np.mean(final_window))) / abs(omega_ref)
    saturation_fraction = float(np.mean(np.abs(voltage) >= 0.99 * V_max))

    return {
        "omega_final": float(omega[-1]),
        "omega_max": omega_max,
        "overshoot": float(overshoot),
        "overshoot_percent": float(100.0 * overshoot),
        "settling_time": settling_time(time, omega, omega_ref, tolerance=tolerance),
        "steady_state_error": float(steady_state_error),
        "steady_state_error_percent": float(100.0 * steady_state_error),
        "voltage_max_abs": float(np.max(np.abs(voltage))),
        "saturation_fraction": saturation_fraction,
        "saturation_percent": float(100.0 * saturation_fraction),
    }


def compute_cost(
    result,
    omega_ref,
    V_max=None,
    vmax=None,
    w_tracking=0.60,
    w_overshoot=0.25,
    w_control=0.15,
):
    """Compute weighted cost function v1."""
    voltage_limit = float(V_max if vmax is None else vmax)
    time = np.asarray(result["time"], dtype=float)
    omega = np.asarray(result["omega"], dtype=float)
    voltage = np.asarray(result["voltage"], dtype=float)

    if omega_ref == 0:
        raise ValueError("omega_ref must be nonzero for normalized metrics.")
    if voltage_limit <= 0:
        raise ValueError("V_max must be positive.")

    T = float(time[-1])
    normalized_error = (omega_ref - omega) / omega_ref
    tracking_cost = _integrate(time * normalized_error**2, time) / T**2

    omega_max = float(np.max(omega))
    overshoot = max(0.0, (omega_max - omega_ref) / omega_ref)
    overshoot_cost = overshoot**2

    normalized_voltage = voltage / voltage_limit
    control_cost = _integrate(normalized_voltage**2, time) / T

    total_cost = (
        w_tracking * tracking_cost
        + w_overshoot * overshoot_cost
        + w_control * control_cost
    )

    metrics = compute_response_metrics(result, omega_ref, voltage_limit)

    return {
        "total": float(total_cost),
        "tracking": float(tracking_cost),
        "overshoot_cost": float(overshoot_cost),
        "control": float(control_cost),
        **metrics,
    }


def accepted_baseline(cost_record, overshoot_limit=0.10, steady_state_limit=0.02):
    """Check v1 baseline acceptance criteria."""
    settling = cost_record.get("settling_time", np.inf)
    return (
        np.isfinite(settling)
        and cost_record["overshoot"] <= overshoot_limit
        and cost_record["steady_state_error"] <= steady_state_limit
        and cost_record["saturation_fraction"] < 0.05
    )
