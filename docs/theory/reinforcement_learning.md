# Reinforcement Learning

이 문서는 `Reinforcement Learning`을 motor control 문제에 어떻게 연결할 수 있는지 정리한다.

v1에서는 특정 RL algorithm을 최종 확정하지 않는다.
먼저 state, action, reward의 개념과 PID optimization과의 차이를 명확히 한다.

---

## 1. Difference from PID Optimization

Classical PID optimization에서는 고정된 PID gains를 찾는다.

```text
Kp
Ki
Kd
```

즉 optimizer의 결과는 다음과 같은 fixed controller이다.

```text
PID Optimization
→ 좋은 고정 PID gains 찾기
```

반면 `Reinforcement Learning`에서는 agent가 현재 motor state를 관찰하고,
매 time step마다 control action을 직접 결정한다.

```text
RL Control
→ 현재 state를 보고 매 순간 action 결정하기
```

---

## 2. State

RL agent가 관찰할 수 있는 state는 motor의 현재 상황을 표현해야 한다.

Possible RL state v1:

$$
s_t
=
\begin{bmatrix}
e_t\\
\omega_t\\
i_t
\end{bmatrix}
$$

각 항의 의미는 다음과 같다.

| State Element | Meaning |
|---|---|
| $e_t$ | Current tracking error |
| $\omega_t$ | Motor speed |
| $i_t$ | Armature current |

필요하면 이후 derivative error, integral error, previous action 등을 state에 추가할 수 있다.

---

## 3. Action

RL agent가 결정하는 action은 우선 motor voltage로 둔다.

$$
a_t=V_t
$$

Simulation에서는 voltage command를 action으로 사용하고,
hardware에서는 이를 `PWM Duty` 형태로 바꿀 수 있다.

```text
Voltage Action
      ↓
Motor Driver
      ↓
PWM Duty
```

---

## 4. Policy

Policy는 현재 state를 입력받아 action을 출력하는 함수이다.

$$
s_t
\rightarrow
\pi_\theta
\rightarrow
a_t
$$

전체 interaction은 다음과 같다.

```text
Current State
      ↓
RL Agent
      ↓
Voltage Action
      ↓
DC Motor
      ↓
Next State
      ↓
Reward
      └────────→ RL Agent
```

---

## 5. Reward

PID optimization에서는 cost를 줄이는 것이 목표이다.

$$
\mathcal{J}\downarrow
$$

RL에서는 reward를 키우는 것이 목표이다.

$$
\mathrm{Reward}\uparrow
$$

Possible reward v1은 다음과 같이 둘 수 있다.

$$
r_t
=
-\alpha\tilde{e}_t^{\,2}
-\beta\tilde{u}_t^{\,2}
$$

즉:

```text
Tracking Error 큼
→ Reward 감소

Control Input 너무 큼
→ Reward 감소
```

Cost minimization과 reward maximization은 표현 방식은 다르지만,
원하는 물리적 목적은 같다.

> **Good Motor Control**
>
> reference speed를 잘 따라가면서, overshoot와 과도한 voltage 사용을 줄이는 것.

---

## 6. Relation to PID Optimization

PID optimization과 RL은 경쟁 관계라기보다 비교 대상이다.

| Method | Controller Form | Main Question |
|---|---|---|
| Manual PID | Fixed PID gains | 사람이 직접 tuning한 baseline은 어느 정도인가 |
| Optimized PID | Fixed PID gains | optimizer가 더 좋은 gains를 찾을 수 있는가 |
| RL Controller | State-dependent policy | fixed gains 없이 action을 직접 결정하면 어떤 차이가 있는가 |

RL 실험은 core PID optimization이 완성된 뒤 진행한다.
초기부터 RL algorithm을 확정하기보다, PID baseline과 optimized PID가 먼저 안정적으로 비교되어야 한다.

---

## 7. Algorithm Decision

현재 문서에서는 최종 RL algorithm을 확정하지 않는다.

후보는 다음과 같이 열어 둔다.

- Discrete action을 쓰는 value-based method
- Continuous voltage action을 쓰는 policy-gradient method
- PID gains를 직접 조정하는 meta-control approach

최종 선택은 simulation environment, action space, training stability를 확인한 뒤 결정한다.
