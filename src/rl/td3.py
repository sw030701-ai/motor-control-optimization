from dataclasses import dataclass

import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
except ImportError as exc:  # pragma: no cover - exercised only when training is requested
    raise ImportError(
        "TD3 training requires PyTorch. Install torch to run the RL experiment."
    ) from exc


@dataclass(frozen=True)
class TD3Config:
    """Training hyperparameters for TD3 direct-voltage control."""

    state_dim: int = 3
    action_dim: int = 1
    max_action: float = 12.0
    discount: float = 0.99
    tau: float = 0.005
    policy_noise: float = 0.20
    noise_clip: float = 0.50
    policy_delay: int = 2
    batch_size: int = 128
    replay_size: int = 200_000
    exploration_noise: float = 2.0
    start_steps: int = 2_000
    hidden_dim: int = 128
    seed: int = 42


class ReplayBuffer:
    """Fixed-size replay buffer for off-policy training."""

    def __init__(self, state_dim, action_dim, max_size, seed=42):
        self.max_size = int(max_size)
        self.ptr = 0
        self.size = 0
        self.rng = np.random.default_rng(seed)
        self.state = np.zeros((self.max_size, state_dim), dtype=np.float32)
        self.action = np.zeros((self.max_size, action_dim), dtype=np.float32)
        self.next_state = np.zeros((self.max_size, state_dim), dtype=np.float32)
        self.reward = np.zeros((self.max_size, 1), dtype=np.float32)
        self.not_done = np.zeros((self.max_size, 1), dtype=np.float32)

    def add(self, state, action, next_state, reward, done):
        self.state[self.ptr] = state
        self.action[self.ptr] = action
        self.next_state[self.ptr] = next_state
        self.reward[self.ptr] = reward
        self.not_done[self.ptr] = 1.0 - float(done)
        self.ptr = (self.ptr + 1) % self.max_size
        self.size = min(self.size + 1, self.max_size)

    def sample(self, batch_size, device):
        idx = self.rng.integers(0, self.size, size=batch_size)
        return (
            torch.as_tensor(self.state[idx], device=device),
            torch.as_tensor(self.action[idx], device=device),
            torch.as_tensor(self.next_state[idx], device=device),
            torch.as_tensor(self.reward[idx], device=device),
            torch.as_tensor(self.not_done[idx], device=device),
        )


class Actor(nn.Module):
    """Deterministic voltage policy."""

    def __init__(self, state_dim, action_dim, max_action, hidden_dim):
        super().__init__()
        self.max_action = float(max_action)
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh(),
        )

    def forward(self, state):
        return self.max_action * self.net(state)


class Critic(nn.Module):
    """Twin Q-functions used by TD3."""

    def __init__(self, state_dim, action_dim, hidden_dim):
        super().__init__()
        input_dim = state_dim + action_dim
        self.q1 = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
        )
        self.q2 = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
        )

    def forward(self, state, action):
        xu = torch.cat([state, action], dim=1)
        return self.q1(xu), self.q2(xu)

    def q1_value(self, state, action):
        return self.q1(torch.cat([state, action], dim=1))


class TD3Agent:
    """Small TD3 implementation for one-dimensional continuous voltage control."""

    def __init__(self, config=None, device=None):
        self.config = config or TD3Config()
        torch.manual_seed(self.config.seed)
        np.random.seed(self.config.seed)
        self.device = device or torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )

        self.actor = Actor(
            self.config.state_dim,
            self.config.action_dim,
            self.config.max_action,
            self.config.hidden_dim,
        ).to(self.device)
        self.actor_target = Actor(
            self.config.state_dim,
            self.config.action_dim,
            self.config.max_action,
            self.config.hidden_dim,
        ).to(self.device)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=3e-4)

        self.critic = Critic(
            self.config.state_dim,
            self.config.action_dim,
            self.config.hidden_dim,
        ).to(self.device)
        self.critic_target = Critic(
            self.config.state_dim,
            self.config.action_dim,
            self.config.hidden_dim,
        ).to(self.device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=3e-4)
        self.total_it = 0

    def select_action(self, state):
        state_tensor = torch.as_tensor(
            state,
            dtype=torch.float32,
            device=self.device,
        ).reshape(1, -1)
        with torch.no_grad():
            action = self.actor(state_tensor).cpu().numpy().reshape(-1)
        return action.astype(float)

    def train(self, replay_buffer):
        self.total_it += 1
        c = self.config
        state, action, next_state, reward, not_done = replay_buffer.sample(
            c.batch_size,
            self.device,
        )

        with torch.no_grad():
            noise = (torch.randn_like(action) * c.policy_noise).clamp(
                -c.noise_clip,
                c.noise_clip,
            )
            next_action = (self.actor_target(next_state) + noise).clamp(
                -c.max_action,
                c.max_action,
            )
            target_q1, target_q2 = self.critic_target(next_state, next_action)
            target_q = torch.min(target_q1, target_q2)
            target_q = reward + not_done * c.discount * target_q

        current_q1, current_q2 = self.critic(state, action)
        critic_loss = F.mse_loss(current_q1, target_q) + F.mse_loss(
            current_q2,
            target_q,
        )
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        actor_loss_value = np.nan
        if self.total_it % c.policy_delay == 0:
            actor_loss = -self.critic.q1_value(state, self.actor(state)).mean()
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()
            actor_loss_value = float(actor_loss.item())

            for param, target_param in zip(
                self.critic.parameters(),
                self.critic_target.parameters(),
            ):
                target_param.data.copy_(
                    c.tau * param.data + (1.0 - c.tau) * target_param.data
                )
            for param, target_param in zip(
                self.actor.parameters(),
                self.actor_target.parameters(),
            ):
                target_param.data.copy_(
                    c.tau * param.data + (1.0 - c.tau) * target_param.data
                )

        return {
            "critic_loss": float(critic_loss.item()),
            "actor_loss": actor_loss_value,
        }

    def save_actor(self, path):
        torch.save(
            {
                "config": self.config.__dict__,
                "state_dict": self.actor.state_dict(),
            },
            path,
        )

    @classmethod
    def load_actor(cls, path, device=None):
        payload = torch.load(path, map_location=device or "cpu")
        agent = cls(config=TD3Config(**payload["config"]), device=device)
        agent.actor.load_state_dict(payload["state_dict"])
        agent.actor_target.load_state_dict(payload["state_dict"])
        return agent


def train_td3_direct_voltage(env, episodes=100, config=None, progress_callback=None):
    """Train TD3 against DirectVoltageMotorEnv and return the agent plus episode history."""
    td3_config = config or TD3Config(max_action=env.config.V_max)
    agent = TD3Agent(td3_config)
    replay_buffer = ReplayBuffer(
        state_dim=td3_config.state_dim,
        action_dim=td3_config.action_dim,
        max_size=td3_config.replay_size,
        seed=td3_config.seed,
    )
    rng = np.random.default_rng(td3_config.seed)
    history = []
    total_steps = 0

    for episode in range(1, episodes + 1):
        observation = env.reset()
        state = env.normalize_observation(observation).astype(np.float32)
        episode_reward = 0.0
        last_losses = {"critic_loss": np.nan, "actor_loss": np.nan}

        for _ in range(env.config.max_steps):
            total_steps += 1
            if total_steps < td3_config.start_steps:
                action = rng.uniform(
                    -env.config.V_max,
                    env.config.V_max,
                    size=td3_config.action_dim,
                )
            else:
                action = agent.select_action(state)
                action += rng.normal(
                    0.0,
                    td3_config.exploration_noise,
                    size=td3_config.action_dim,
                )
                action = np.clip(action, -env.config.V_max, env.config.V_max)

            next_observation, reward, done, info = env.step(action)
            next_state = env.normalize_observation(next_observation).astype(np.float32)
            replay_buffer.add(state, action, next_state, reward, done)
            state = next_state
            episode_reward += reward

            if replay_buffer.size >= td3_config.batch_size:
                last_losses = agent.train(replay_buffer)
            if done:
                break

        record = {
            "episode": episode,
            "episode_reward": float(episode_reward),
            "final_omega": float(info["omega"]),
            "final_error": float(info["error"]),
            **last_losses,
        }
        history.append(record)
        if progress_callback is not None:
            progress_callback(record)

    return agent, history
