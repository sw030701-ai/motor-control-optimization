import math

import numpy as np

from src.motor.dc_motor import nominal_dc_motor_params
from src.optimization.cost_function import compute_cost
from src.rl.direct_voltage_env import (
    DirectVoltageEnvConfig,
    DirectVoltageMotorEnv,
    simulate_direct_voltage_policy,
)


def test_direct_voltage_env_uses_error_speed_current_state():
    params = nominal_dc_motor_params()
    config = DirectVoltageEnvConfig(omega_ref=12.6, V_max=12.0, simulation_time=0.01)
    env = DirectVoltageMotorEnv(params, config=config)

    observation = env.reset()

    assert observation.shape == (3,)
    assert np.allclose(observation, [12.6, 0.0, 0.0])


def test_direct_voltage_env_clips_action_to_voltage_limit():
    params = nominal_dc_motor_params()
    config = DirectVoltageEnvConfig(omega_ref=12.6, V_max=12.0, simulation_time=0.01)
    env = DirectVoltageMotorEnv(params, config=config)
    env.reset()

    _, _, _, info = env.step(100.0)

    assert info["voltage"] == 12.0


def test_direct_voltage_reward_matches_tracking_overshoot_control_weights():
    params = nominal_dc_motor_params()
    config = DirectVoltageEnvConfig(omega_ref=10.0, V_max=12.0)
    env = DirectVoltageMotorEnv(params, config=config)

    reward = env._reward(omega=11.0, voltage=6.0)
    expected = -(
        0.60 * ((10.0 - 11.0) / 10.0) ** 2
        + 0.25 * ((11.0 - 10.0) / 10.0) ** 2
        + 0.15 * (6.0 / 12.0) ** 2
    )

    assert math.isclose(reward, expected)


def test_direct_voltage_policy_result_reuses_cost_metrics_shape():
    params = nominal_dc_motor_params()
    config = DirectVoltageEnvConfig(
        omega_ref=12.6,
        V_max=12.0,
        simulation_time=0.02,
        dt=0.001,
    )

    result = simulate_direct_voltage_policy(params, lambda state: 0.0, config=config)
    cost = compute_cost(result, omega_ref=config.omega_ref, V_max=config.V_max)

    assert {"time", "omega", "current", "voltage", "error"}.issubset(result)
    assert {"total", "tracking", "overshoot_cost", "control"}.issubset(cost)
