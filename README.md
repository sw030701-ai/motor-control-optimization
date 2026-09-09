
# Motor Control Optimization

## Project Goal

This project aims to optimize motor control performance using
classical optimization and reinforcement learning.

The project starts with a DC motor speed control system and may later
be extended to mobile robot control.

## Main Pipeline

1. DC motor modeling
2. PID baseline controller
3. Cost function design
4. Classical optimization
5. Reinforcement learning control
6. Performance comparison
7. Robustness test
8. Hardware / mobile robot extension

## Optimization Methods

- Manual PID tuning
- Random Search
- Genetic Algorithm
- Bayesian Optimization
- Reinforcement Learning

## Optional Experimental Extension

- QUBO formulation
- QAOA
- Warm-start QAOA
- Comparison with classical optimization

## Project Structure

- `src/motor/` : motor models
- `src/controller/` : PID and control algorithms
- `src/optimization/` : optimization methods
- `src/rl/` : reinforcement learning controller
- `src/simulation/` : simulation code
- `experiments/` : experiment notebooks
- `results/` : figures and tables
- `docs/` : theory and project notes
- `tests/` : test code

## Current Status

Baseline PID experiment is available:

- Motor model notebook: `experiments/01_dc_motor_model.ipynb`
- Notebook: `experiments/02_pid_baseline_tuning.ipynb`
- PID optimization notebook: `experiments/03_pid_optimization.ipynb`
- RL direct voltage control script: `experiments/04_rl_direct_voltage_control.py`
- Nominal motor source: literature-based parameter set, DOI `10.1177/00202940261442256`
- Baseline gains: `K_p = 0.80`, `K_i = 2.00`, `K_d = 0.002`
- Record: `results/tables/baseline_pid_tuning_record.md`

## RL Direct Voltage Control

The RL experiment uses direct voltage control instead of PID gain tuning:

```text
[e_t, omega_t, i_t] -> TD3 agent -> V_t -> DC motor
```

`V_t` is clipped to the actuator range `[-12 V, 12 V]`. TD3 is used because
the action is continuous and one-dimensional, so a deterministic policy with
exploration noise is simpler for this project than SAC while still matching
the direct-voltage control problem.

Run the experiment from the repository root:

```bash
python experiments/04_rl_direct_voltage_control.py --episodes 100
```

PyTorch is required for TD3 training:

```bash
pip install torch
```

The script writes:

- `results/tables/rl_direct_voltage_training_history.csv`
- `results/tables/rl_direct_voltage_evaluation.csv`
- `results/tables/direct_voltage_rl_comparison.csv`
- `results/figures/rl_direct_voltage_speed_comparison.png`
- `results/figures/rl_direct_voltage_control_comparison.png`

Initial 100-episode TD3 result:

| Controller | J_total | Steady-state error [%] | Feasible |
|---|---:|---:|---|
| Manual PID | 0.03818 | 0.00000 | True |
| Optimized PID | 0.03670 | 0.00000 | True |
| RL Direct Control | 0.07485 | 45.84141 | False |

In this v1 setting, RL Direct Control did not replace optimized PID. The result
is kept as evidence that direct RL control needs more careful training design
before it is useful for this DC motor speed-control problem.
