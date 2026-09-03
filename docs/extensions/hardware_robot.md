# Hardware / Robot Extension

이 문서는 simulation에서 검증한 controller를 실제 hardware와 robot control로 확장하는 방향을 정리한다.

Hardware extension은 core simulation project가 안정적으로 완성된 뒤 진행한다.

---

## 1. Simulation-to-Hardware Path

Simulation 단계에서는 motor voltage $V(t)$를 직접 control input으로 사용한다.

실제 hardware에서는 이 값이 motor driver의 `PWM Duty`로 변환된다.

```text
Simulation
Voltage Command V(t)
      ↓

Hardware
PWM Duty
      ↓
Motor Driver
      ↓
DC Motor
```

---

## 2. Possible Hardware Components

| Component | Role |
|---|---|
| Arduino / STM32 | Controller implementation |
| Motor driver | PWM signal을 motor voltage / current로 변환 |
| DC motor | Controlled plant |
| Encoder | Motor speed measurement |
| Current sensor | Armature current measurement, optional |
| Power supply | Motor power |

초기 hardware test에서는 current sensor 없이 encoder speed feedback만 사용해도 된다.
다만 current까지 측정하면 simulation model과 실제 motor response를 더 잘 비교할 수 있다.

---

## 3. Physical Control Loop

실제 system의 control loop는 다음과 같다.

```text
Reference Speed
      ↓
Controller
      ↓
PWM / Voltage Command
      ↓
Motor Driver
      ↓
DC Motor
      ↓
Encoder
      ↓
Measured Speed
      └──────────── Feedback
```

Simulation에서 사용한 PID gains를 hardware에 바로 적용할 수 있지만,
실제 motor에서는 friction, dead zone, sensor noise, voltage drop 때문에 retuning이 필요할 수 있다.

---

## 4. Embedded Implementation Notes

Hardware implementation에서는 다음을 고려한다.

- Sampling time을 일정하게 유지한다.
- Encoder pulse를 speed로 변환한다.
- Voltage command를 PWM duty로 saturation한다.
- Integral windup을 방지한다.
- Derivative term에는 filtering을 고려한다.
- Emergency stop 또는 safe voltage limit을 둔다.

---

## 5. Differential-Drive Robot Extension

DC motor 하나의 speed control이 성공하면,
두 개의 motor를 사용하는 `Differential Drive Robot`으로 확장할 수 있다.

Left wheel speed:

```math
\omega_L
```

Right wheel speed:

```math
\omega_R
```

확장 흐름은 다음과 같다.

```text
Single Motor Speed Control
          ↓
Left / Right Motor Control
          ↓
Robot Velocity Control
          ↓
Robot Motion Control
          ↓
Trajectory Tracking
```

---

## 6. Long-Term Direction

최종적으로는 단순 motor speed control이 아니라 trajectory tracking 문제로 확장할 수 있다.

```math
\text{Trajectory Tracking}
```

이 단계에서는 motor-level PID control과 robot-level motion control을 구분해야 한다.

| Level | Control Target |
|---|---|
| Motor level | $\omega_L$, $\omega_R$ |
| Robot level | Linear velocity, angular velocity, position |

Robot extension은 core PID optimization과 RL comparison이 완료된 뒤 진행한다.
