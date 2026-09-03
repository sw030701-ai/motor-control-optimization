# Robustness Test

이 문서는 nominal condition에서 좋은 controller가 실제 환경 변화에서도 안정적으로 동작하는지 확인하는 실험 계획이다.

---

## 1. Goal

Nominal motor model에서 얻은 controller가 parameter variation, disturbance, sensor noise에도 잘 동작하는지 확인한다.

실제 motor에서는 parameter가 정확히 일정하지 않을 수 있다.

원인 예시:

- Motor temperature
- Load change
- Mechanical friction
- Parameter uncertainty
- Sensor noise

---

## 2. Benchmark Logic

Robustness test에서도 comparison logic은 동일하다.

```text
Conventional PID
      vs
Optimized PID
      vs
RL Controller
```

중요한 점은 controller를 robustness condition마다 새로 tuning하지 않는 것이다.

```text
Tune under nominal condition
      ↓
Fix controller
      ↓
Evaluate under changed condition
```

그래야 "nominal에서 찾은 controller가 환경 변화에도 robust한가?"를 확인할 수 있다.

---

## 3. Resistance Variation

Motor temperature가 변하면 armature resistance가 변할 수 있다.

Example:

$$
R\rightarrow1.2R
$$

또는:

$$
R\rightarrow0.8R
$$

확인할 질문:

- Response speed가 크게 변하는가?
- Steady-state error가 증가하는가?
- Control input이 saturation에 더 자주 걸리는가?

---

## 4. Rotor Inertia Variation

Motor shaft에 load가 연결되면 effective inertia가 증가할 수 있다.

Example:

$$
J_m\rightarrow1.3J_m
$$

확인할 질문:

- Settling time이 얼마나 증가하는가?
- Overshoot가 증가하거나 감소하는가?
- Controller가 느려진 plant에도 안정적인가?

---

## 5. Load Torque Disturbance

Nominal simulation에서는:

$$
T_L=0
$$

으로 시작하지만 robustness test에서는:

$$
T_L\neq0
$$

조건을 적용한다.

예를 들어 motor가 일정 speed로 회전하는 도중 갑자기 load torque를 적용한다.

```text
Normal Operation
      ↓
Sudden Load Torque
      ↓
Speed Drop
      ↓
Controller Recovery
```

확인할 질문:

- Load torque 직후 speed drop이 얼마나 큰가?
- Reference speed로 얼마나 빨리 복귀하는가?
- Recovery 중 voltage saturation이 지속되는가?

---

## 6. Sensor Noise

실제 encoder measurement에는 noise가 존재할 수 있다.

Measured speed를 다음과 같이 구성할 수 있다.

$$
\omega_{\mathrm{measured}}
=
\omega+\mathrm{noise}
$$

확인할 질문:

- Controller output $V(t)$가 noise 때문에 지나치게 흔들리는가?
- Derivative term이 noise를 증폭시키는가?
- Filtering 또는 derivative 제한이 필요한가?

---

## 7. Robustness Table

| Test Condition | Conventional PID | Optimized PID | RL Controller | Notes |
|---|---:|---:|---:|---|
| Nominal | TBD | TBD | TBD | Baseline condition |
| $R\rightarrow1.2R$ | TBD | TBD | TBD | Resistance increase |
| $R\rightarrow0.8R$ | TBD | TBD | TBD | Resistance decrease |
| $J_m\rightarrow1.3J_m$ | TBD | TBD | TBD | Inertia increase |
| Load torque step | TBD | TBD | TBD | Disturbance recovery |
| Sensor noise | TBD | TBD | TBD | Measurement robustness |

---

## 8. Main Question

Robustness test의 핵심 질문은 다음과 같다.

> **Nominal Motor Model에서 찾은 controller가 motor parameter와 외란이 변해도 안정적으로 동작하는가?**

Nominal cost가 가장 낮은 controller가 반드시 robustness에서도 가장 좋지는 않을 수 있다.
따라서 final comparison에서는 nominal performance와 robustness를 함께 본다.
