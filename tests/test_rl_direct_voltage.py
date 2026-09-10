import math

import numpy as np
import pytest

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


def test_td3_config_uses_reduced_actor_critic_learning_rates():
    pytest.importorskip("torch")

    from src.rl.td3 import TD3Agent, TD3Config

    config = TD3Config()
    agent = TD3Agent(config)

    assert config.actor_lr == 1e-4
    assert config.critic_lr == 1e-4
    assert agent.actor_optimizer.param_groups[0]["lr"] == config.actor_lr
    assert agent.critic_optimizer.param_groups[0]["lr"] == config.critic_lr


def test_rl_direct_voltage_script_defaults_to_100_episodes(monkeypatch):
    import importlib.util
    from pathlib import Path

    script_path = Path(__file__).resolve().parents[1] / "experiments" / "04_rl_direct_voltage_control.py"
    spec = importlib.util.spec_from_file_location("rl_direct_voltage_script", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr("sys.argv", ["04_rl_direct_voltage_control.py"])

    args = module.parse_args()

    assert args.episodes == 100
    assert args.eval_every == 10
    assert args.actor_lr == 1e-4
    assert args.critic_lr == 1e-4


def test_td3_training_saves_best_deterministic_eval_checkpoint(tmp_path):
    pytest.importorskip("torch")

    from src.rl.td3 import TD3Agent, TD3Config, train_td3_direct_voltage

    params = nominal_dc_motor_params()
    env_config = DirectVoltageEnvConfig(
        omega_ref=12.6,
        V_max=12.0,
        simulation_time=0.001,
        dt=0.001,
    )
    env = DirectVoltageMotorEnv(params, config=env_config)
    checkpoint_path = tmp_path / "best_actor.pt"

    def deterministic_evaluation(_agent, episode):
        return {
            "J_total": 2.0 if episode == 1 else 1.0,
            "steady_state_error_percent": 0.0,
        }

    _, history, evaluation_history, best_evaluation = train_td3_direct_voltage(
        env,
        episodes=2,
        config=TD3Config(
            max_action=env_config.V_max,
            batch_size=1,
            start_steps=0,
            hidden_dim=8,
            actor_lr=1e-4,
            critic_lr=1e-4,
        ),
        evaluation_callback=deterministic_evaluation,
        eval_every=1,
        checkpoint_path=checkpoint_path,
        checkpoint_metric="J_total",
    )

    metadata = TD3Agent.load_actor_metadata(checkpoint_path)

    assert checkpoint_path.exists()
    assert len(history) == 2
    assert len(evaluation_history) == 2
    assert best_evaluation["episode"] == 2
    assert metadata["checkpoint_metric"] == "J_total"
    assert metadata["checkpoint_episode"] == 2
    assert metadata["actor_lr"] == 1e-4
    assert metadata["critic_lr"] == 1e-4
    assert metadata["selection_rule"] == "lowest deterministic evaluation J_total"
