
import numpy as np


def compute_cost(
    result,
    omega_ref,
    vmax,
    w_tracking=0.60,
    w_overshoot=0.25,
    w_control=0.15
):
    """
    Cost Function v1

    J =
        0.60 * tracking cost
        + 0.25 * overshoot cost
        + 0.15 * control effort
    """

    time = result["time"]
    omega = result["omega"]
    voltage = result["voltage"]

    T = time[-1]

    # -----------------------------------
    # 1. Tracking Cost
    # Normalized ITSE
    # -----------------------------------

    normalized_error = (
        omega_ref - omega
    ) / omega_ref

    tracking_integrand = (
        time * normalized_error**2
    )

    tracking_cost = (
        np.trapezoid(
            tracking_integrand,
            time
        )
        / T**2
    )

    # -----------------------------------
    # 2. Overshoot Cost
    # -----------------------------------

    omega_max = np.max(omega)

    overshoot = max(
        0.0,
        (omega_max - omega_ref)
        / omega_ref
    )

    overshoot_cost = overshoot**2

    # -----------------------------------
    # 3. Control Effort
    # -----------------------------------

    normalized_voltage = (
        voltage / vmax
    )

    control_cost = (
        np.trapezoid(
            normalized_voltage**2,
            time
        )
        / T
    )

    # -----------------------------------
    # Total Cost
    # -----------------------------------

    total_cost = (
        w_tracking * tracking_cost
        + w_overshoot * overshoot_cost
        + w_control * control_cost
    )

    return {
        "total": total_cost,
        "tracking": tracking_cost,
        "overshoot": overshoot_cost,
        "control": control_cost
    }
