# RL Direct Voltage Control Experiment

이 실험은 PID gain tuning을 하지 않는다.
RL agent가 motor state를 직접 관찰하고 매 time step마다 voltage command를 출력한다.

```text
[e_t, omega_t, i_t] -> TD3 Agent -> V_t -> DC Motor
```

## 고정 실험 조건

기존 main 실험과 같은 nominal DC motor, reference speed, voltage limit을 사용한다.

| Item | Value |
|---|---|
| Motor | `nominal_dc_motor_params()` |
| Reference | `12.6 rad/s` |
| Voltage limit | `-12 V <= V_t <= 12 V` |
| Evaluation metrics | `compute_cost()` and `is_feasible_v1()` |

## Training / Evaluation 분리

Training episode에서는 10초 simulation 동안 exploration noise를 더하고
replay buffer update와 actor/critic network update를 수행한다.
Evaluation에서는 exploration을 끄고 deterministic actor output만 사용하며,
같은 10초 simulation 조건에서 replay buffer 저장과 network update를 하지 않는다.

기본 설정은 매 `10` episode마다 deterministic evaluation을 수행한다.
Evaluation도 기존 motor 초기조건, `reference = 12.6 rad/s`, 10초 simulation,
기존 `compute_cost()` metric을 그대로 사용한다.
Best checkpoint는 deterministic evaluation에서 `J_total`이 가장 낮은 policy로
선택한다. 따라서 최종 comparison의 RL row는 마지막 training episode가 아니라
`results/models/td3_direct_voltage_best_actor.pt`를 다시 로드해서 계산한다.

Actor와 critic learning rate는 초기 구현의 `3e-4`에서 각각 `1e-4`로 낮췄다.
이는 episode 후반에 policy가 크게 움직이며 성능이 흔들릴 수 있는 현상을 줄이기
위한 보수적인 안정화 설정이다.

## State

```math
s_t=
\begin{bmatrix}
e_t\\
\omega_t\\
i_t
\end{bmatrix}
```

| Element | 의미 |
|---|---|
| $e_t$ | $\omega_{ref}-\omega_t$ |
| $\omega_t$ | 현재 motor speed |
| $i_t$ | armature current |

## Action

```math
a_t=V_t
```

environment는 action을 다음 범위로 clip한다.

```math
V_t=\mathrm{clip}(a_t,-12,12)
```

## Reward

reward는 기존 PID optimization cost의 철학을 그대로 따른다.

```math
r_t=
-\left[
0.60\left(\frac{e_t}{\omega_{ref}}\right)^2
+0.25\left(\max\left(0,\frac{\omega_t-\omega_{ref}}{\omega_{ref}}\right)\right)^2
+0.15\left(\frac{V_t}{V_{max}}\right)^2
\right]
```

따라서 최종 비교에서도 기존 tracking, overshoot, control effort metric과
같은 기준으로 해석할 수 있다.

## Algorithm

구현은 `TD3`를 사용한다.
direct voltage control은 action space가 continuous이고, action dimension이 1개이다.
그래서 deterministic policy가 voltage를 직접 출력하는 TD3가 이 v1 실험에 자연스럽다.
SAC도 가능하지만, 현재 프로젝트에서는 TD3가 구현 의존성과 tuning 부담이 더 작다.

## 실행 방법

notebook에서 결과 표와 그래프를 확인한다.

```bash
jupyter notebook experiments/04_rl_direct_voltage_control.ipynb
```

같은 실험을 script로 다시 실행할 수도 있다.

```bash
python experiments/04_rl_direct_voltage_control.py --episodes 100
```

script는 최종 comparison table을 저장한다.

```text
results/tables/direct_voltage_rl_comparison.csv
```

추가로 evaluation checkpoint history와 actor checkpoint를 저장한다.

```text
results/tables/rl_direct_voltage_eval_history.csv
results/models/td3_direct_voltage_best_actor.pt
results/models/td3_direct_voltage_last_actor.pt
```

비교 대상은 다음과 같다.

| Controller | 의미 |
|---|---|
| Manual PID | 사람이 정한 baseline PID gains |
| Optimized PID | main optimization에서 찾은 constrained PID |
| RL Direct Control | TD3 policy가 voltage를 직접 출력하는 controller |

## 초기 결과

초기 100-episode TD3 실행 결과, RL Direct Control은 v1 feasibility criteria를
만족하지 못했다.

| Controller | J_total | Steady-state error [%] | Saturation [%] | Feasible |
|---|---:|---:|---:|---|
| Manual PID | 0.03818 | 0.00000 | 0.000 | True |
| Optimized PID | 0.03670 | 0.00000 | 0.000 | True |
| RL Direct Control | 0.07485 | 45.84141 | 0.180 | False |

학습된 RL policy는 target speed보다 낮은 속도에 머물렀다.

```text
omega_final = 6.824 rad/s
omega_ref   = 12.600 rad/s
```

이 결과는 현재의 작은 training budget과 단순한 state 정의에서는 TD3 direct voltage
control이 constrained PID보다 안정적이지 않다는 것을 보여준다.
따라서 RL 결과는 실패가 아니라, 이 DC motor setup에서 direct RL control을 쓰려면
추가적인 reward shaping, 학습 안정화, 검증 절차가 필요하다는 한계 분석으로 해석한다.
