from dataclasses import dataclass

import numpy as np

from src.motor.dc_motor import DCMotor


@dataclass(frozen=True)
class DirectVoltageEnvConfig:
    """Configuration for direct-voltage RL motor control."""

    omega_ref: float = 12.6
    V_max: float = 12.0
    simulation_time: float = 10.0
    dt: float = 0.001
    load_torque: float = 0.0
    method: str = "rk4"
    w_tracking: float = 0.60
    w_overshoot: float = 0.25
    w_control: float = 0.15

    @property
    def max_steps(self):
        return int(round(self.simulation_time / self.dt)) + 1


class DirectVoltageMotorEnv:
    """
    Minimal continuous-action RL environment for DC motor speed control.

    Observation:
        [e_t, omega_t, i_t]
    Action:
        voltage command clipped to [-V_max, V_max]
    """

    def __init__(self, motor_params, config=None):
        self.motor_params = motor_params
        self.config = config or DirectVoltageEnvConfig()
        self.motor = DCMotor(motor_params)
        self.step_index = 0
        self.last_voltage = 0.0

    def reset(self, current=0.0, omega=0.0):
        self.motor.reset(current=current, omega=omega)
        self.step_index = 0
        self.last_voltage = 0.0
        return self._observation()

    def _observation(self):
        error = self.config.omega_ref - self.motor.omega
        return np.array([error, self.motor.omega, self.motor.current], dtype=float)

    def normalize_observation(self, observation):
        """Scale [error, speed, current] for neural-network training."""
        observation = np.asarray(observation, dtype=float)
        current_scale = max(
            self.config.V_max / max(self.motor_params.R, 1e-12),
            1.0,
        )
        return np.array(
            [
                observation[0] / self.config.omega_ref,
                observation[1] / self.config.omega_ref,
                observation[2] / current_scale,
            ],
            dtype=float,
        )

    def _reward(self, omega, voltage):
        error_norm = (self.config.omega_ref - omega) / self.config.omega_ref
        overshoot_norm = max(
            0.0,
            (omega - self.config.omega_ref) / self.config.omega_ref,
        )
        voltage_norm = voltage / self.config.V_max
        return -float(
            self.config.w_tracking * error_norm**2
            + self.config.w_overshoot * overshoot_norm**2
            + self.config.w_control * voltage_norm**2
        )

    def step(self, action):
        voltage = float(
            np.clip(
                np.asarray(action).reshape(-1)[0],
                -self.config.V_max,
                self.config.V_max,
            )
        )
        current, omega = self.motor.step(
            voltage=voltage,
            dt=self.config.dt,
            load_torque=self.config.load_torque,
            method=self.config.method,
        )
        self.step_index += 1
        self.last_voltage = voltage
        observation = self._observation()
        reward = self._reward(omega=omega, voltage=voltage)
        done = self.step_index >= self.config.max_steps
        info = {
            "voltage": voltage,
            "current": float(current),
            "omega": float(omega),
            "error": float(observation[0]),
        }
        return observation, reward, done, info


def simulate_direct_voltage_policy(motor_params, policy, config=None):
    """
    Simulate a direct-voltage policy and return the same result shape as simulate_pid().

    The policy receives the raw RL state [e_t, omega_t, i_t] and returns voltage.
    """
    env = DirectVoltageMotorEnv(motor_params, config=config)
    observation = env.reset()
    time = np.arange(
        0.0,
        env.config.simulation_time + 0.5 * env.config.dt,
        env.config.dt,
    )

    omega_history = np.zeros_like(time)
    current_history = np.zeros_like(time)
    voltage_history = np.zeros_like(time)
    error_history = np.zeros_like(time)
    reward_history = np.zeros_like(time)

    for k, _ in enumerate(time):
        voltage = policy(observation)
        observation, reward, done, info = env.step(voltage)
        omega_history[k] = info["omega"]
        current_history[k] = info["current"]
        voltage_history[k] = info["voltage"]
        error_history[k] = info["error"]
        reward_history[k] = reward
        if done and k < len(time) - 1:
            omega_history[k + 1 :] = info["omega"]
            current_history[k + 1 :] = info["current"]
            voltage_history[k + 1 :] = info["voltage"]
            error_history[k + 1 :] = info["error"]
            reward_history[k + 1 :] = reward
            break

    return {
        "time": time,
        "omega": omega_history,
        "current": current_history,
        "voltage": voltage_history,
        "error": error_history,
        "reward": reward_history,
        "omega_ref": env.config.omega_ref,
        "V_max": env.config.V_max,
    }
