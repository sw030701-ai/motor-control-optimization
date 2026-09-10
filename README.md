
# Motor Control Optimization

## Project Goal

이 프로젝트는 classical optimization과 reinforcement learning을 사용해
motor control 성능을 비교하고 최적화하는 것을 목표로 한다.

첫 단계에서는 DC motor speed control system을 다루고,
이후 mobile robot control로 확장할 수 있다.

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

현재 가능한 실험은 다음과 같다.

- Motor model notebook: `experiments/01_dc_motor_model.ipynb`
- Notebook: `experiments/02_pid_baseline_tuning.ipynb`
- PID optimization notebook: `experiments/03_pid_optimization.ipynb`
- RL direct voltage control notebook: `experiments/04_rl_direct_voltage_control.ipynb`
- RL direct voltage control script: `experiments/04_rl_direct_voltage_control.py`
- Nominal motor source: literature-based parameter set, DOI `10.1177/00202940261442256`
- Baseline gains: `K_p = 0.80`, `K_i = 2.00`, `K_d = 0.002`
- Record: `results/tables/baseline_pid_tuning_record.md`

## RL Direct Voltage Control

RL 실험은 PID gain tuning이 아니라 direct voltage control 방식으로 진행한다.

```text
[e_t, omega_t, i_t] -> TD3 agent -> V_t -> DC motor
```

`V_t`는 actuator limit에 맞춰 `[-12 V, 12 V]` 범위로 clip한다.
TD3를 사용한 이유는 action이 1차원 continuous voltage이기 때문이다.
이 문제에서는 deterministic policy에 exploration noise를 더하는 TD3 구조가
SAC보다 구현이 단순하고 현재 프로젝트 규모에 더 자연스럽다.

notebook에서 결과 표와 response plot을 확인할 수 있다.

```bash
jupyter notebook experiments/04_rl_direct_voltage_control.ipynb
```

같은 실험을 script로 다시 실행할 수도 있다.

```bash
python experiments/04_rl_direct_voltage_control.py --episodes 100
```

TD3 학습에는 PyTorch가 필요하다.

```bash
pip install torch
```

script는 다음 결과 파일을 저장한다.

- `results/tables/rl_direct_voltage_training_history.csv`
- `results/tables/rl_direct_voltage_evaluation.csv`
- `results/tables/direct_voltage_rl_comparison.csv`
- `results/figures/rl_direct_voltage_speed_comparison.png`
- `results/figures/rl_direct_voltage_control_comparison.png`

초기 100-episode TD3 결과는 다음과 같다.

| Controller | J_total | Steady-state error [%] | Feasible |
|---|---:|---:|---|
| Manual PID | 0.03818 | 0.00000 | True |
| Optimized PID | 0.03670 | 0.00000 | True |
| RL Direct Control | 0.07485 | 45.84141 | False |

이 v1 설정에서는 RL Direct Control이 Optimized PID를 대체하지 못했다.
따라서 이 결과는 direct RL control이 현재 state, reward, training budget에서는
한계가 있음을 보여주는 비교 결과로 해석한다.
