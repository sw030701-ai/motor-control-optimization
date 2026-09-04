# Baseline PID Tuning

이 문서는 conventional / manual `PID Baseline`을 만드는 절차를 정리한다.

Baseline PID의 목적은 optimizer와 RL controller가 비교할 기준점을 만드는 것이다.
따라서 baseline은 일부러 나쁘게 만들지 않고, 안정적으로 동작하는 reasonable controller로 설정한다.

---

## 1. Baseline Role

전체 비교 흐름은 다음과 같다.

```text
Manual PID Baseline
      ↓
Optimized PID
      ↓
RL Controller
```

Baseline PID는 다음 질문에 답하기 위한 기준이다.

> Optimization 또는 RL을 사용했을 때, 사람이 직접 tuning한 conventional PID보다 실제로 좋아졌는가?

---

## 2. Nominal Conditions

Baseline tuning은 먼저 nominal simulation condition에서 수행한다.

| Item | v1 Setting |
|---|---|
| Motor model | Nominal DC motor model |
| Load torque | $T_L=0$ |
| Reference input | Step reference $\omega_{\mathrm{ref}}$ |
| Voltage limit | $\pm V_{\max}$ |
| Simulation time | TBD |
| Sampling time | TBD |
| Initial speed | $\omega(0)=0$ |
| Initial current | $i(0)=0$ |

Nominal parameter set은 [Open-Loop Validation](open_loop_validation.md) 이후 확정한다.

---

## 3. Reference Speed Selection

Reference speed는 motor가 voltage limit 안에서 도달 가능한 값이어야 한다.

No-load steady-state maximum speed를 다음과 같이 계산한다.

```math
\omega_{ss,\max}
=
\frac{V_{\max}}{K_e+\frac{Rb}{K_t}}
```

v1에서는 reference speed를 다음 범위에서 시작한다.

```math
\omega_{\mathrm{ref}}
\approx
0.5\sim0.8
\times
\omega_{ss,\max}
```

Reference가 너무 높으면 PID가 아무리 좋아도 voltage saturation 때문에 target을 따라가지 못한다.
그 경우 tuning 문제와 actuator limit 문제가 섞여버린다.

---

## 4. Sequential Manual Tuning

Manual PID baseline은 sequential tuning으로 만든다.

```text
Step 1: Ki = 0, Kd = 0으로 두고 Kp tuning
Step 2: Kp를 고정한 뒤 Ki를 조금씩 증가
Step 3: 필요한 경우 Kd를 작게 추가
Step 4: acceptance criteria를 만족하면 gains 고정
```

---

## 5. Step 1 - Tune Kp

먼저 integral과 derivative gain을 0으로 둔다.

```math
K_i=0,\quad K_d=0
```

이 상태에서 $K_p$를 작게 시작해서 점진적으로 증가시킨다.

```text
Kp too small
→ response가 너무 느림

Kp reasonable
→ reference에 빠르게 접근하지만 안정적

Kp too large
→ overshoot 또는 oscillation 증가
```

이 단계의 목표는 motor response가 reference를 향해 충분히 빠르게 움직이되,
sustained oscillation이 발생하지 않는 $K_p$ 후보를 찾는 것이다.

---

## 6. Step 2 - Add Ki

다음으로 $K_p$를 고정하고 $K_i$를 조금씩 증가시킨다.

`Ki`의 목적은 steady-state error를 줄이는 것이다.

```text
Ki too small
→ steady-state error가 남음

Ki reasonable
→ steady-state error 감소

Ki too large
→ overshoot, slow recovery, integral windup 가능
```

이 단계에서는 final speed가 reference 근처에 수렴하는지 확인한다.

---

## 7. Step 3 - Add Small Kd if Needed

Overshoot가 크거나 response가 너무 흔들리면 $K_d$를 작게 추가한다.

`Kd`는 error 변화율에 반응하므로 damping 역할을 할 수 있다.

```text
Kd small
→ overshoot 감소 가능

Kd too large
→ noise sensitivity 증가, control input이 거칠어질 수 있음
```

Simulation에서는 noise가 작아 보이더라도,
hardware extension에서는 encoder noise 때문에 derivative term이 문제가 될 수 있다.
따라서 v1 baseline에서는 $K_d$를 필요할 때만 작게 사용한다.

---

## 8. Acceptance Criteria

Baseline PID는 다음 조건을 만족해야 한다.

| Criterion | v1 Target |
|---|---:|
| Stability | Divergence 없음 |
| Sustained oscillation | 없어야 함 |
| Overshoot | $\le 10\%$ |
| Steady-state error | $\le 2\%$ |
| Voltage saturation | Persistent saturation 피하기 |

여기서 `10% overshoot`와 `2% steady-state error`는 v1 target이다.
나중에 motor parameter나 reference speed가 바뀌면 기준을 조정할 수 있다.

---

## 9. Fix Baseline Gains

Acceptance criteria를 만족하면 baseline gains를 고정한다.

```math
K_{p,baseline},\quad
K_{i,baseline},\quad
K_{d,baseline}
```

이후 comparison에서는 baseline gains를 계속 바꾸지 않는다.

```text
Tune baseline once
      ↓
Fix baseline gains
      ↓
Compare with optimized PID and RL under same conditions
```

Baseline을 comparison 중간에 계속 조정하면,
optimized PID와 RL controller의 성능 차이를 공정하게 비교하기 어렵다.

---

## 10. Baseline Cost

Baseline gains를 고정한 뒤, 동일한 cost function으로 baseline cost를 계산한다.

```math
\mathcal{J}_{baseline}
=
\mathcal{J}
\left(
K_{p,baseline},
K_{i,baseline},
K_{d,baseline}
\right)
```

이 값은 benchmark로만 사용한다.

```text
J_baseline
→ comparison 기준

Optimizer
→ J_baseline 없이도 실행 가능
```

Optimizer가 필요로 하는 것은 각 candidate gains의 cost이다.
Baseline cost는 optimizer를 돌리기 위한 필수 입력이 아니다.

---

## 11. Why Baseline Should Not Be Deliberately Poor

일부러 나쁜 baseline을 만들면 optimized PID나 RL controller가 쉽게 좋아 보인다.
하지만 그 비교는 의미가 약하다.

```text
Poor baseline
      ↓
Optimizer가 좋아 보임
      ↓
하지만 실제 improvement인지 판단하기 어려움
```

좋은 실험 비교는 다음과 같아야 한다.

```text
Reasonable manual PID
      ↓
Optimized PID가 얼마나 개선하는가?
      ↓
RL Controller는 어떤 장단점이 있는가?
```

따라서 baseline PID는 안정적이고 납득 가능한 conventional controller로 만든다.

---

## 12. Tuning Record Template

| Item | Value |
|---|---|
| $K_{p,baseline}$ | `0.80` |
| $K_{i,baseline}$ | `2.00` |
| $K_{d,baseline}$ | `0.002` |
| Reference speed | `12.60 rad/s` |
| Overshoot | `0.0 %` |
| Steady-state error | `~0 %` |
| Settling time | `1.322 s` |
| Control effort | `0.2541` |
| $\mathcal{J}_{baseline}$ | `0.03818` |

이 값은 [02 PID Baseline Tuning Notebook](../../experiments/02_pid_baseline_tuning.ipynb)에서 계산했다.
전체 record는 [baseline_pid_tuning_record.md](../../results/tables/baseline_pid_tuning_record.md)에 저장한다.

이 값을 고정한 뒤 [Optimization Plan](optimization_plan.md)으로 넘어간다.
