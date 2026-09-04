# AI-Based Motor Control Parameter Optimization

> **Project Plan v1**  
> DC Motor Speed Control을 시작으로 `PID Optimization`, `Reinforcement Learning`,
> `Robustness Test`, 실제 `Hardware / Robot Control`까지 확장하는 프로젝트.

---

## 1. Project Goal

본 프로젝트의 목표는 `DC Motor Speed Control` 문제에서 controller parameter를
체계적으로 설계하고 비교하는 것이다.

첫 단계에서는 conventional `PID Controller`를 baseline으로 만들고,
이후 `Random Search`, `Bayesian Optimization`, 그리고 `Reinforcement Learning`
기반 controller와 성능을 비교한다.

최종적으로는 simulation에서 검증한 방법을 실제 motor hardware와 mobile robot으로
확장할 수 있는 구조를 만드는 것을 목표로 한다.

---

## 2. Research Questions

이 프로젝트에서 확인하고 싶은 핵심 질문은 다음과 같다.

1. DC motor speed control에서 manual PID baseline은 어느 정도 성능을 보이는가?
2. Classical optimization은 PID gains를 manual tuning보다 더 안정적으로 찾을 수 있는가?
3. Cost function의 구성은 optimized PID의 성능과 control effort에 어떤 영향을 주는가?
4. Nominal condition에서 찾은 controller는 parameter variation과 disturbance에도 robust한가?
5. RL controller는 fixed-gain PID optimization과 어떤 차이를 보이는가?

---

## 3. Project Pipeline

전체 프로젝트 흐름은 다음과 같다.

```text
Motor Modeling
      ↓
Open-Loop Validation
      ↓
PID Baseline
      ↓
Cost Function Design
      ↓
Classical Optimization
      ↓
Performance Comparison
      ↓
Reinforcement Learning
      ↓
Robustness Test
      ↓
Hardware / Robot Extension
```

핵심 비교 흐름은 다음 순서로 둔다.

```text
Manual PID
      ↓
Random Search
      ↓
Bayesian Optimization
      ↓
RL Controller
```

`Genetic Algorithm`은 main story가 아니라 optional comparison으로 사용한다.

---

## 4. System Scope

현재 v1의 제어 대상은 `Armature-Controlled DC Motor`의 angular speed이다.

| Item | v1 Scope |
|---|---|
| Target System | Armature-Controlled DC Motor |
| Control Target | Angular Speed $\omega(t)$ |
| Control Input | Voltage $V(t)$ |
| States | Armature Current $i(t)$, Angular Speed $\omega(t)$ |
| Disturbance | Load Torque $T_L(t)$ |
| Baseline Controller | Conventional PID Controller |
| Optimization Variables | $K_p$, $K_i$, $K_d$ |

Motor model의 자세한 식과 notation은 [DC Motor Modeling](theory/motor_model.md)에 정리한다.

---

## 5. Method Overview

### Motor Modeling

DC motor의 electrical dynamics와 mechanical dynamics를 state-space model로 구성한다.
초기 simulation에서는 nominal parameter set과 $T_L=0$ 조건으로 시작한다.

Detailed document:

- [DC Motor Modeling](theory/motor_model.md)
- [Open-Loop Validation](experiments/open_loop_validation.md)

### PID Baseline

Conventional PID controller를 직접 tuning하여 baseline controller로 사용한다.
Baseline은 이후 optimized PID와 RL controller를 비교하기 위한 기준점이다.

Detailed document:

- [PID Control Theory](theory/pid_control.md)
- [Baseline PID Tuning](experiments/baseline_pid_tuning.md)

### Cost Function

좋은 control response를 하나의 numerical score로 평가하기 위해 weighted cost function을 사용한다.
v1에서는 tracking error, overshoot, control effort를 함께 평가한다.

Detailed document:

- [Cost Function Design](theory/cost_function.md)

### Classical Optimization

Optimizer는 PID gains candidate를 선택하고, simulation 결과로 계산된 cost를 다시 받아
더 좋은 gains를 찾는다.

Detailed document:

- [Classical Optimization](theory/optimization.md)
- [Optimization Plan](experiments/optimization_plan.md)

### Reinforcement Learning

RL은 fixed PID gains를 찾는 방식이 아니라, motor state를 보고 매 time step마다 action을
결정하는 방식으로 확장한다. v1에서는 개념과 비교 방향을 정리하고, 최종 알고리즘은
실험 단계에서 확정한다.

Detailed document:

- [Reinforcement Learning](theory/reinforcement_learning.md)

---

## 6. Evaluation Strategy

모든 controller는 가능한 한 동일한 motor model, reference input, simulation time,
voltage limit, sampling condition에서 평가한다.

주요 metric은 다음과 같다.

- `Total Cost`
- `Tracking Error`
- `Overshoot`
- `Settling Time`
- `Steady-State Error`
- `Control Effort`

Detailed document:

- [Performance Comparison](experiments/performance_comparison.md)

---

## 7. Robustness Strategy

Nominal condition에서 얻은 controller가 실제 환경 변화에서도 안정적인지 확인한다.

Robustness test에서는 다음 조건을 고려한다.

- Armature resistance variation
- Rotor inertia variation
- Load torque disturbance
- Sensor noise

Detailed document:

- [Robustness Test](experiments/robustness_test.md)

---

## 8. Extensions

Simulation 검증 이후에는 실제 embedded hardware와 robot control로 확장한다.
Quantum optimization은 core project가 완성된 이후 optional experiment로만 다룬다.

Detailed documents:

- [Hardware / Robot Extension](extensions/hardware_robot.md)
- [Optional Quantum Optimization](extensions/quantum_optimization.md)

---

## 9. Milestones

| Step | Milestone | Status |
|---:|---|---|
| 1 | Nominal DC motor parameters 결정 | Done |
| 2 | Motor model 구현 및 open-loop validation | Done |
| 3 | Manual PID baseline tuning | Done |
| 4 | Cost function 구현 | Done |
| 5 | Random Search 기반 PID optimization | TBD |
| 6 | Bayesian Optimization 기반 PID optimization | TBD |
| 7 | Conventional PID vs Optimized PID 비교 | TBD |
| 8 | RL controller 실험 | TBD |
| 9 | Robustness test | TBD |
| 10 | Hardware / robot extension 검토 | TBD |

---

## 10. Current v1 Decisions

```text
Control Target
└─ Angular Speed ω(t)

Motor Model
├─ Armature Current i(t)
└─ Angular Speed ω(t)

Baseline Controller
└─ Conventional PID Controller

Optimization Variables
├─ Kp
├─ Ki
└─ Kd

Cost Function
├─ Tracking Error
├─ Overshoot
└─ Control Effort

Main Optimization Story
├─ Manual PID
├─ Random Search
└─ Bayesian Optimization

Optional / Extension
├─ Genetic Algorithm
├─ Reinforcement Learning
├─ Hardware / Robot Control
└─ Quantum Optimization
```

---

## 11. Detailed Documentation

문서 전체 인덱스는 [docs/README.md](README.md)를 기준으로 관리한다.
