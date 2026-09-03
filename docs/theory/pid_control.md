# PID Control Theory

이 문서는 `DC Motor Speed Control`에서 사용할 conventional `PID Controller`의
기본 개념을 정리한다.

실제 manual tuning 절차는 [Baseline PID Tuning](../experiments/baseline_pid_tuning.md)에 둔다.

---

## 1. Feedback Control

PID controller는 feedback을 사용한다.

```text
Reference Speed
      ↓
Tracking Error
      ↓
PID Controller
      ↓
Voltage Command
      ↓
DC Motor
      ↓
Measured Speed
      └──────── Feedback
```

목표 speed와 실제 speed를 계속 비교하면서 voltage command를 조정한다.

---

## 2. Tracking Error

Reference speed를 $\omega_{\mathrm{ref}}(t)$,
실제 motor speed를 $\omega(t)$라고 하면 tracking error는 다음과 같다.

```math
e(t)=\omega_{\mathrm{ref}}(t)-\omega(t)
```

즉:

```text
e(t) > 0  → 실제 speed가 reference보다 낮음
e(t) < 0  → 실제 speed가 reference보다 높음
e(t) = 0  → reference를 정확히 추종
```

---

## 3. PID Control Law

PID controller의 output은 motor에 입력되는 voltage command로 둔다.

```math
V_{\mathrm{cmd}}(t)
=
K_pe(t)
+
K_i\int_0^t e(\tau)\,d\tau
+
K_d\frac{de(t)}{dt}
```

실제 motor에 들어가는 voltage는 saturation을 적용한 값이다.

```math
V(t)=\mathrm{sat}\left(V_{\mathrm{cmd}}(t), -V_{\max}, V_{\max}\right)
```

---

## 4. PID Gains

PID에서 조정해야 하는 세 parameter는 다음과 같다.

```text
Kp
Ki
Kd
```

이 세 값을 `PID Gains`라고 한다.

| Gain | Name | Main Role |
|---|---|---|
| $K_p$ | Proportional gain | 현재 error에 즉시 반응 |
| $K_i$ | Integral gain | steady-state error 제거 |
| $K_d$ | Derivative gain | error 변화율을 이용해 damping 제공 |

---

## 5. Kp Role

`Kp`는 현재 error의 크기에 비례해서 control input을 만든다.

```text
현재 Error 큼
      ↓
Kp × Error 큼
      ↓
강한 Control Input
```

목표 speed와 실제 speed 차이가 클수록 더 강하게 motor를 가속하거나 감속한다.

단, `Kp`가 지나치게 크면 response가 aggressive해지고,
overshoot 또는 oscillation이 커질 수 있다.

---

## 6. Ki Role

`Ki`는 과거부터 누적된 error를 사용한다.

```math
K_i\int_0^t e(\tau)\,d\tau
```

작은 error가 오랫동안 남아 있으면 integral term이 계속 누적된다.
따라서 `Ki`는 주로 `Steady-State Error`를 줄이는 역할을 한다.

하지만 `Ki`가 너무 크면 overshoot, slow recovery, integral windup이 발생할 수 있다.

---

## 7. Kd Role

`Kd`는 error의 변화 속도에 반응한다.

```math
K_d\frac{de(t)}{dt}
```

Speed가 너무 빠르게 reference를 넘어가려는 경우 response를 완화하는 damping 역할을 할 수 있다.

다만 derivative term은 noise에 민감하다.
따라서 encoder noise가 있는 실제 hardware에서는 filtering이 필요할 수 있다.

---

## 8. Saturation and Anti-Windup

실제 motor driver는 무한한 voltage를 낼 수 없다.
따라서 controller output은 반드시 voltage limit 안에 있어야 한다.

```text
V_cmd >  V_max  →  V =  V_max
V_cmd < -V_max  →  V = -V_max
```

Saturation 상태에서 integral term이 계속 누적되면 `Integral Windup`이 발생할 수 있다.

v1에서는 다음 원칙을 둔다.

- Baseline PID tuning에서는 persistent voltage saturation을 피한다.
- Optimized PID 평가에서도 control effort cost로 과도한 voltage 사용을 penalty로 둔다.
- Hardware extension에서는 anti-windup logic을 별도로 검토한다.

---

## 9. Practical Notes

PID controller는 단순하지만 비교 기준으로 매우 중요하다.

```text
Manual PID
      ↓
Optimized PID
      ↓
RL Controller
```

이 순서로 비교해야 optimization과 learning method가 실제로 baseline보다 나아졌는지 판단할 수 있다.

Baseline이 너무 나쁘면 optimized method가 좋아 보이는 착시가 생기고,
baseline이 지나치게 공격적이면 robustness test에서 불안정해질 수 있다.

따라서 baseline PID는 "일부러 못 만든 controller"가 아니라,
reasonable하고 안정적인 conventional controller로 설정한다.
