# Documentation Index

이 폴더는 `AI-Based Motor Control Parameter Optimization` 프로젝트의 문서를 관리한다.

`project_plan.md`는 전체 방향을 빠르게 파악하기 위한 high-level plan이고,
수식, tuning 절차, experiment detail은 아래 세부 문서로 나누어 정리한다.

---

## Project Overview

| Document | Description |
|---|---|
| [Project Plan](project_plan.md) | 프로젝트 목표, pipeline, scope, milestone 요약 |

---

## Theory

| Document | Description |
|---|---|
| [DC Motor Modeling](theory/motor_model.md) | DC motor electrical / mechanical equation, state-space model, notation |
| [PID Control Theory](theory/pid_control.md) | Feedback, tracking error, PID equation, gain 역할, saturation |
| [Cost Function Design](theory/cost_function.md) | Tracking, overshoot, control effort, weighted cost function |
| [Classical Optimization](theory/optimization.md) | PID gain optimization objective, Random Search, GA, Bayesian Optimization |
| [Reinforcement Learning](theory/reinforcement_learning.md) | RL state, action, reward 개념과 PID optimization과의 관계 |

---

## Experiments

| Document | Description |
|---|---|
| [Open-Loop Validation](experiments/open_loop_validation.md) | Motor parameter 선정, open-loop sanity check, reachable reference speed |
| [Baseline PID Tuning](experiments/baseline_pid_tuning.md) | Conventional / manual PID baseline tuning procedure |
| [Optimization Plan](experiments/optimization_plan.md) | PID gain search bounds, evaluation budget, 동일 조건 비교 계획 |
| [Performance Comparison](experiments/performance_comparison.md) | Conventional PID, Optimized PID, RL 비교 table skeleton |
| [Robustness Test](experiments/robustness_test.md) | Parameter variation, load torque, sensor noise benchmark |

---

## Extensions

| Document | Description |
|---|---|
| [Hardware / Robot Extension](extensions/hardware_robot.md) | Arduino / STM32, motor driver, encoder, differential-drive robot 확장 |
| [Optional Quantum Optimization](extensions/quantum_optimization.md) | PID gain discretization, binary encoding, QUBO, QAOA extension |

---

## Documentation Rule

새로운 내용을 추가할 때는 다음 기준을 따른다.

| Content Type | Recommended Location |
|---|---|
| 프로젝트 전체 목표와 흐름 | `docs/project_plan.md` |
| 수식과 이론 설명 | `docs/theory/` |
| 실험 절차와 조건 | `docs/experiments/` |
| 장기 확장 아이디어 | `docs/extensions/` |

`project_plan.md`에는 derivation, tuning manual, detailed experiment log를 넣지 않는다.
