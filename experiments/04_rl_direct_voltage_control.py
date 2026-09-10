"""
RL Direct Voltage Control 실험.

motor input은 1차원 continuous voltage action이므로 TD3를 사용한다.
TD3는 deterministic voltage policy를 학습하고, training 중 exploration noise를
더하는 방식이라 SAC보다 현재 v1 실험에 필요한 구현 부담이 작다.

이 controller는 PID gain tuner가 아니다.
[e_t, omega_t, i_t]를 관찰하고 V_t를 직접 출력하며,
V_t는 actuator limit인 [-12 V, 12 V] 범위로 clip한다.
"""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

PROJECT_ROOT = Path(__file__).resolve().parents[1]

import sys

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.controller.pid import PIDGains
from src.motor.dc_motor import nominal_dc_motor_params
from src.optimization.cost_function import compute_cost, is_feasible_v1
from src.rl.direct_voltage_env import (
    DirectVoltageEnvConfig,
    DirectVoltageMotorEnv,
    simulate_direct_voltage_policy,
)
from src.simulation.pid_simulation import reference_from_reachable_speed, simulate_pid


RESULT_TABLE_DIR = PROJECT_ROOT / "results" / "tables"
RESULT_FIGURE_DIR = PROJECT_ROOT / "results" / "figures"
RESULT_MODEL_DIR = PROJECT_ROOT / "results" / "models"

BASELINE_RECORD_PATH = RESULT_TABLE_DIR / "baseline_pid_tuning_record.csv"
OPTIMIZED_RECORD_PATH = RESULT_TABLE_DIR / "constrained_pid_optimization_summary.csv"
TRAINING_HISTORY_PATH = RESULT_TABLE_DIR / "rl_direct_voltage_training_history.csv"
EVAL_HISTORY_PATH = RESULT_TABLE_DIR / "rl_direct_voltage_eval_history.csv"
RL_EVALUATION_PATH = RESULT_TABLE_DIR / "rl_direct_voltage_evaluation.csv"
COMPARISON_PATH = RESULT_TABLE_DIR / "direct_voltage_rl_comparison.csv"
SUMMARY_PATH = RESULT_TABLE_DIR / "rl_direct_voltage_summary.json"
BEST_ACTOR_PATH = RESULT_MODEL_DIR / "td3_direct_voltage_best_actor.pt"
LAST_ACTOR_PATH = RESULT_MODEL_DIR / "td3_direct_voltage_last_actor.pt"
CHECKPOINT_METRIC = "J_total"


def _json_safe(value):
    if isinstance(value, dict):
        return {key: _json_safe(val) for key, val in value.items()}
    if isinstance(value, list):
        return [_json_safe(val) for val in value]
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if pd.isna(value):
        return None
    return value


def _load_fixed_conditions(params):
    if BASELINE_RECORD_PATH.exists():
        baseline = pd.read_csv(BASELINE_RECORD_PATH).iloc[0]
        return float(baseline["reference_speed_rad_s"]), float(baseline["V_max"])
    V_max = 12.0
    omega_ref = round(reference_from_reachable_speed(params, V_max, fraction=0.50), 2)
    return omega_ref, V_max


def _load_manual_gains():
    if BASELINE_RECORD_PATH.exists():
        row = pd.read_csv(BASELINE_RECORD_PATH).iloc[0]
        return PIDGains(
            K_p=float(row["K_p_baseline"]),
            K_i=float(row["K_i_baseline"]),
            K_d=float(row["K_d_baseline"]),
        )
    return PIDGains(K_p=0.80, K_i=2.00, K_d=0.002)


def _load_optimized_gains():
    if not OPTIMIZED_RECORD_PATH.exists():
        return None
    table = pd.read_csv(OPTIMIZED_RECORD_PATH)
    optimized = table[table["Controller"].isin(["Constrained Classical", "Optimized PID"])]
    if optimized.empty:
        return None
    row = optimized.iloc[0]
    return PIDGains(K_p=float(row["K_p"]), K_i=float(row["K_i"]), K_d=float(row["K_d"]))


def _controller_row(controller, result, omega_ref, V_max, gains=None):
    cost = compute_cost(result, omega_ref=omega_ref, V_max=V_max)
    row = {
        "Controller": controller,
        "K_p": np.nan,
        "K_i": np.nan,
        "K_d": np.nan,
        "J_total": cost["total"],
        "J_tracking": cost["tracking"],
        "J_overshoot": cost["overshoot_cost"],
        "J_control": cost["control"],
        "overshoot_percent": cost["overshoot_percent"],
        "settling_time": cost["settling_time"],
        "steady_state_error_percent": cost["steady_state_error_percent"],
        "voltage_max_abs": cost["voltage_max_abs"],
        "saturation_percent": cost["saturation_percent"],
        "omega_final": cost["omega_final"],
        "feasible": is_feasible_v1(cost),
    }
    if gains is not None:
        row.update(gains.as_dict())
    return row


def _evaluate_pid_controller(name, gains, params, omega_ref, V_max, simulation_time, dt):
    result = simulate_pid(
        motor_params=params,
        gains=gains,
        omega_ref=omega_ref,
        V_max=V_max,
        simulation_time=simulation_time,
        dt=dt,
    )
    return _controller_row(name, result, omega_ref, V_max, gains=gains), result


def _evaluate_agent(agent, params, eval_env_config, episode=None):
    eval_env = DirectVoltageMotorEnv(params, config=eval_env_config)

    def td3_policy(observation):
        normalized = eval_env.normalize_observation(observation)
        return agent.select_action(normalized)[0]

    result = simulate_direct_voltage_policy(params, td3_policy, config=eval_env_config)
    row = _controller_row(
        "RL Direct Control",
        result,
        eval_env_config.omega_ref,
        eval_env_config.V_max,
    )
    if episode is not None:
        row["episode"] = episode
    return row, result


def _load_or_train_agent(args, params, train_env_config, eval_env_config):
    from src.rl.td3 import TD3Agent, TD3Config, train_td3_direct_voltage

    td3_config = TD3Config(
        max_action=train_env_config.V_max,
        batch_size=args.batch_size,
        start_steps=args.start_steps,
        exploration_noise=args.exploration_noise,
        seed=args.seed,
        actor_lr=args.actor_lr,
        critic_lr=args.critic_lr,
    )

    if args.evaluate_only:
        if not BEST_ACTOR_PATH.exists():
            raise FileNotFoundError(
                f"No saved best actor found at {BEST_ACTOR_PATH}. Run without --evaluate-only first."
            )
        metadata = TD3Agent.load_actor_metadata(BEST_ACTOR_PATH)
        return TD3Agent.load_actor(BEST_ACTOR_PATH), [], [], metadata

    env = DirectVoltageMotorEnv(params, config=train_env_config)

    def progress(record):
        if record["episode"] == 1 or record["episode"] % args.log_every == 0:
            eval_text = ""
            if f"eval_{CHECKPOINT_METRIC}" in record:
                eval_text = (
                    f" eval_{CHECKPOINT_METRIC}="
                    f"{record[f'eval_{CHECKPOINT_METRIC}']:.5f}"
                )
            print(
                f"episode={record['episode']:4d} "
                f"reward={record['episode_reward']:10.3f} "
                f"final_error={record['final_error']:8.4f}"
                f"{eval_text}"
            )

    def deterministic_evaluation(agent, episode):
        row, _ = _evaluate_agent(agent, params, eval_env_config, episode=episode)
        return {
            key: value
            for key, value in row.items()
            if key not in {"Controller", "K_p", "K_i", "K_d"}
        }

    agent, history, evaluation_history, best_evaluation = train_td3_direct_voltage(
        env=env,
        episodes=args.episodes,
        config=td3_config,
        progress_callback=progress,
        evaluation_callback=deterministic_evaluation,
        eval_every=args.eval_every,
        checkpoint_path=BEST_ACTOR_PATH,
        checkpoint_metric=CHECKPOINT_METRIC,
    )
    RESULT_MODEL_DIR.mkdir(parents=True, exist_ok=True)
    agent.save_actor(
        LAST_ACTOR_PATH,
        metadata={
            "episodes": args.episodes,
            "note": "Last training actor; final comparison uses the best deterministic evaluation checkpoint.",
        },
    )
    if best_evaluation is None:
        agent.save_actor(
            BEST_ACTOR_PATH,
            metadata={
                "checkpoint_episode": args.episodes,
                "checkpoint_metric": CHECKPOINT_METRIC,
                "checkpoint_score": None,
                "selection_rule": f"lowest deterministic evaluation {CHECKPOINT_METRIC}",
                "note": "No scheduled evaluation was run; saved final actor as fallback.",
            },
        )
        best_evaluation = {
            "checkpoint_episode": args.episodes,
            "checkpoint_metric": CHECKPOINT_METRIC,
            "checkpoint_score": None,
        }

    best_agent = TD3Agent.load_actor(BEST_ACTOR_PATH)
    best_metadata = TD3Agent.load_actor_metadata(BEST_ACTOR_PATH)
    return best_agent, history, evaluation_history, best_metadata


def _plot_response(results_by_controller, omega_ref, V_max):
    RESULT_FIGURE_DIR.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(9, 5))
    for name, result in results_by_controller.items():
        ax.plot(result["time"], result["omega"], label=name)
    ax.axhline(omega_ref, color="black", linestyle="--", linewidth=1, label="Reference")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [rad/s]")
    ax.set_title("Manual PID vs Optimized PID vs RL Direct Voltage Control")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(RESULT_FIGURE_DIR / "rl_direct_voltage_speed_comparison.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 5))
    for name, result in results_by_controller.items():
        ax.plot(result["time"], result["voltage"], label=name)
    ax.axhline(V_max, color="black", linestyle="--", linewidth=1)
    ax.axhline(-V_max, color="black", linestyle="--", linewidth=1)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Voltage [V]")
    ax.set_title("Control Voltage Comparison")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(RESULT_FIGURE_DIR / "rl_direct_voltage_control_comparison.png", dpi=160)
    plt.close(fig)


def _plot_training_history(history, evaluation_history):
    if not history:
        return
    RESULT_FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    history_df = pd.DataFrame(history)

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(
        history_df["episode"],
        history_df["episode_reward"],
        label="training reward",
    )
    axes[0].set_ylabel("Episode reward")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(
        history_df["episode"],
        history_df["final_error"],
        label="training final error",
    )
    if evaluation_history:
        eval_df = pd.DataFrame(evaluation_history)
        axes[1].plot(
            eval_df["episode"],
            eval_df[CHECKPOINT_METRIC],
            marker="o",
            label=f"deterministic eval {CHECKPOINT_METRIC}",
        )
    axes[1].set_xlabel("Episode")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.suptitle("TD3 Training History and Deterministic Evaluations")
    fig.tight_layout()
    fig.savefig(RESULT_FIGURE_DIR / "rl_direct_voltage_training_history.png", dpi=160)
    plt.close(fig)


def run(args):
    RESULT_TABLE_DIR.mkdir(parents=True, exist_ok=True)
    params = nominal_dc_motor_params()
    omega_ref, V_max = _load_fixed_conditions(params)

    train_env_config = DirectVoltageEnvConfig(
        omega_ref=omega_ref,
        V_max=V_max,
        simulation_time=args.train_simulation_time,
        dt=args.train_dt,
    )
    eval_env_config = DirectVoltageEnvConfig(
        omega_ref=omega_ref,
        V_max=V_max,
        simulation_time=args.eval_simulation_time,
        dt=args.eval_dt,
    )

    agent, history, evaluation_history, best_metadata = _load_or_train_agent(
        args,
        params,
        train_env_config,
        eval_env_config,
    )
    if history:
        pd.DataFrame(history).to_csv(TRAINING_HISTORY_PATH, index=False)
        _plot_training_history(history, evaluation_history)
    if evaluation_history:
        pd.DataFrame(evaluation_history).to_csv(EVAL_HISTORY_PATH, index=False)

    rl_row, rl_result = _evaluate_agent(agent, params, eval_env_config)
    rl_row.update(
        {
            "checkpoint_type": "best_deterministic_eval",
            "checkpoint_metric": best_metadata.get("checkpoint_metric", CHECKPOINT_METRIC),
            "checkpoint_score": best_metadata.get("checkpoint_score"),
            "checkpoint_episode": best_metadata.get("checkpoint_episode"),
        }
    )
    pd.DataFrame([rl_row]).to_csv(RL_EVALUATION_PATH, index=False)

    rows = []
    results = {}

    manual_gains = _load_manual_gains()
    row, result = _evaluate_pid_controller(
        "Manual PID",
        manual_gains,
        params,
        omega_ref,
        V_max,
        args.eval_simulation_time,
        args.eval_dt,
    )
    rows.append(row)
    results["Manual PID"] = result

    optimized_gains = _load_optimized_gains()
    if optimized_gains is not None:
        row, result = _evaluate_pid_controller(
            "Optimized PID",
            optimized_gains,
            params,
            omega_ref,
            V_max,
            args.eval_simulation_time,
            args.eval_dt,
        )
        rows.append(row)
        results["Optimized PID"] = result

    rows.append(rl_row)
    results["RL Direct Control"] = rl_result

    comparison = pd.DataFrame(rows)
    baseline_j = float(comparison.loc[comparison["Controller"] == "Manual PID", "J_total"].iloc[0])
    comparison["J_improvement_vs_manual_percent"] = (
        (baseline_j - comparison["J_total"]) / baseline_j * 100.0
    )
    comparison.to_csv(COMPARISON_PATH, index=False)
    _plot_response(results, omega_ref=omega_ref, V_max=V_max)

    summary = {
        "algorithm": "TD3",
        "algorithm_reason": (
            "TD3 is used because direct motor voltage is a continuous action; "
            "it keeps the implementation smaller than SAC for this 1D deterministic policy."
        ),
        "state": ["e_t", "omega_t", "i_t"],
        "action": "V_t clipped to [-V_max, V_max]",
        "reward_weights": {
            "tracking": eval_env_config.w_tracking,
            "overshoot": eval_env_config.w_overshoot,
            "control_effort": eval_env_config.w_control,
        },
        "fixed_conditions": {
            "omega_ref": omega_ref,
            "V_max": V_max,
            "train_dt": args.train_dt,
            "eval_dt": args.eval_dt,
            "train_simulation_time": args.train_simulation_time,
            "eval_simulation_time": args.eval_simulation_time,
        },
        "training_evaluation_split": {
            "training": "exploration noise and network updates are enabled",
            "evaluation": "deterministic action selection with exploration and network updates disabled",
            "eval_every_episodes": args.eval_every,
        },
        "learning_rates": {
            "previous_actor_lr": 3e-4,
            "previous_critic_lr": 3e-4,
            "actor_lr": args.actor_lr,
            "critic_lr": args.critic_lr,
            "reason": (
                "Both learning rates were reduced to make actor and critic updates "
                "more conservative after observed training instability."
            ),
        },
        "best_checkpoint": {
            "path": str(BEST_ACTOR_PATH),
            "selection_metric": CHECKPOINT_METRIC,
            "selection_rule": f"lowest deterministic evaluation {CHECKPOINT_METRIC}",
            "metadata": best_metadata,
        },
        "outputs": {
            "training_history": str(TRAINING_HISTORY_PATH),
            "eval_history": str(EVAL_HISTORY_PATH),
            "rl_evaluation": str(RL_EVALUATION_PATH),
            "comparison": str(COMPARISON_PATH),
            "best_actor": str(BEST_ACTOR_PATH),
            "last_actor": str(LAST_ACTOR_PATH),
        },
        "comparison": comparison.to_dict(orient="records"),
    }
    with open(SUMMARY_PATH, "w", encoding="utf-8") as f:
        json.dump(_json_safe(summary), f, indent=2, ensure_ascii=False)

    print("\nSaved RL comparison:")
    print(comparison)
    print(f"\nComparison table: {COMPARISON_PATH}")
    print(f"Best actor checkpoint: {BEST_ACTOR_PATH}")
    print(f"Best checkpoint criterion: lowest deterministic evaluation {CHECKPOINT_METRIC}")


def parse_args():
    parser = argparse.ArgumentParser(description="Train and evaluate TD3 direct voltage control.")
    parser.add_argument("--episodes", type=int, default=30)
    parser.add_argument("--train-simulation-time", type=float, default=10.0)
    parser.add_argument("--train-dt", type=float, default=0.005)
    parser.add_argument("--eval-simulation-time", type=float, default=10.0)
    parser.add_argument("--eval-dt", type=float, default=0.001)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--start-steps", type=int, default=2_000)
    parser.add_argument("--exploration-noise", type=float, default=2.0)
    parser.add_argument("--actor-lr", type=float, default=1e-4)
    parser.add_argument("--critic-lr", type=float, default=1e-4)
    parser.add_argument("--eval-every", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--log-every", type=int, default=5)
    parser.add_argument("--evaluate-only", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
