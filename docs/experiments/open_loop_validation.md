# Open-Loop Validation

이 문서는 PID controller를 연결하기 전에 motor model 자체가 정상적으로 동작하는지 확인하는 절차를 정리한다.

---

## 1. Why Motor Parameters First?

PID controller를 설계하기 전에 먼저 어떤 motor를 제어할 것인지 정해야 한다.

현재 DC motor model은 다음 equation으로 정의되어 있다.

$$
V(t)
=
L\frac{di(t)}{dt}
+
Ri(t)
+
K_e\omega(t)
$$

$$
J_m\frac{d\omega(t)}{dt}
=
K_ti(t)
-b\omega(t)
-T_L(t)
$$

Simulation을 수행하려면 다음 parameter들의 numerical value가 필요하다.

```text
R   = Armature Resistance
L   = Armature Inductance
J_m = Rotor Inertia
b   = Viscous Friction
Kt  = Torque Constant
Ke  = Back-EMF Constant
```

이 값들은 PID controller가 결정하는 값이 아니라, 제어 대상인 motor 자체의 물리적 특성이다.

```text
Motor Parameters
R, L, J_m, b, Kt, Ke
        ↓
"어떤 Motor를 제어하는가?"

PID Gains
Kp, Ki, Kd
        ↓
"그 Motor를 어떻게 제어하는가?"
```

Nominal motor parameter는 실제 DC motor datasheet 또는 신뢰할 수 있는 reference model을 기준으로 선정한다.

---

## 2. Open-Loop Concept

Open-loop validation에서는 feedback controller를 사용하지 않는다.

```text
Input Voltage V(t)
        ↓
    DC Motor
        ↓
Motor Speed ω(t)
```

이 단계에서는 다음과 같다.

```text
PID Controller = 없음
Feedback       = 없음
```

예를 들어 motor에 일정한 step voltage를 입력한다.

$$
V(t)=V_{\mathrm{test}}
$$

그러면 simulation에서 armature current $i(t)$와 motor speed $\omega(t)$가 물리적으로 그럴듯한 response를 보이는지 확인한다.

---

## 3. Theoretical Steady-State Speed

부하가 없는 nominal condition에서:

$$
T_L=0
$$

steady-state에서는 derivative가 0이다.

$$
\frac{di(t)}{dt}=0,\quad
\frac{d\omega(t)}{dt}=0
$$

Electrical equation:

$$
V=Ri+K_e\omega
$$

Mechanical equation:

$$
0=K_ti-b\omega
$$

따라서:

$$
i=\frac{b}{K_t}\omega
$$

이를 electrical equation에 대입하면:

$$
V=
R\frac{b}{K_t}\omega
+
K_e\omega
$$

따라서 theoretical steady-state speed는 다음과 같다.

$$
\omega_{ss}
=
\frac{V}{K_e+\frac{Rb}{K_t}}
$$

즉 open-loop simulation의 최종 speed는 이 값과 가까워야 한다.

---

## 4. Model Sanity Checks

Open-loop response에서 확인할 항목은 다음과 같다.

| Check | Expected Behavior |
|---|---|
| Current response | 초기에는 current가 증가하고, speed 증가에 따라 back EMF 때문에 안정화 |
| Speed response | step voltage 입력 후 점진적으로 steady-state speed에 접근 |
| Steady-state speed | $\omega_{ss}=V/(K_e+Rb/K_t)$와 큰 차이가 없어야 함 |
| No unstable growth | feedback 없이도 passive motor model이 발산하지 않아야 함 |
| Sign consistency | positive voltage에 대해 positive speed가 나와야 함 |

만약 open-loop response가 이상하면 PID tuning으로 넘어가기 전에 model equation,
parameter unit, numerical integration setting을 먼저 확인한다.

---

## 5. Reachable Speed and Reference Selection

PID reference speed는 motor가 실제로 도달 가능한 값이어야 한다.

Voltage limit이 $V_{\max}$일 때 no-load steady-state speed는:

$$
\omega_{ss,\max}
=
\frac{V_{\max}}{K_e+\frac{Rb}{K_t}}
$$

따라서 initial reference speed는 다음 원칙으로 선택한다.

```text
ω_ref < ω_ss,max
```

너무 높은 reference를 선택하면 controller가 계속 saturation에 걸리고,
PID tuning 결과가 motor 성능이 아니라 actuator limit에 의해 결정된다.

v1에서는 안전하게 다음 범위에서 시작한다.

$$
\omega_{\mathrm{ref}}
\approx
0.5\sim0.8
\times
\omega_{ss,\max}
$$

---

## 6. Outputs

Open-loop validation 후 기록할 값은 다음과 같다.

| Item | Value |
|---|---|
| Nominal parameter set | TBD |
| Test voltage $V_{\mathrm{test}}$ | TBD |
| Theoretical $\omega_{ss}$ | TBD |
| Simulated final speed | TBD |
| Difference | TBD |
| Selected $\omega_{\mathrm{ref}}$ | TBD |

이 결과가 정상적이면 [Baseline PID Tuning](baseline_pid_tuning.md)으로 진행한다.
