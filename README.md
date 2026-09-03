
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

- Notebook: `experiments/02_pid_baseline_tuning.ipynb`
- Baseline gains: `K_p = 0.08`, `K_i = 0.80`, `K_d = 0.002`
- Record: `results/tables/baseline_pid_tuning_record.md`
