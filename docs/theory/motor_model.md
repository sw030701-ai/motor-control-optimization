# DC Motor Modeling

이 문서는 프로젝트에서 사용할 `Armature-Controlled DC Motor` model을 정리한다.

목표는 실제 motor가 없어도 Python simulation 안에서

```text
Input Voltage V(t)
        ↓
DC Motor Model
        ↓
Current i(t)
Angular Speed ω(t)
```

와 같은 response를 계산할 수 있도록 수학적인 motor model을 만드는 것이다.

---

## 1. Model Overview

현재 프로젝트의 제어 대상은 motor speed이다.

| Symbol | Meaning |
|---|---|
| $V(t)$ | Applied motor voltage |
| $i(t)$ | Armature current |
| $\omega(t)$ | Angular velocity |
| $T_L(t)$ | Load torque disturbance |
| $y(t)$ | Output speed |

`Input`은 voltage $V(t)$이고, `Output`은 angular speed $\omega(t)$이다.

---

## 2. Electrical Dynamics

DC motor의 armature circuit에 `KVL`을 적용하면 다음과 같다.

```math
V(t)
=
L\frac{di(t)}{dt}
+
Ri(t)
+
K_e\omega(t)
```

이를 current derivative 형태로 정리하면:

```math
\frac{di(t)}{dt}
=
-\frac{R}{L}i(t)
-\frac{K_e}{L}\omega(t)
+\frac{1}{L}V(t)
```

여기서 $K_e\omega(t)$는 `Back EMF`이다.

```math
e_b(t)=K_e\omega(t)
```

Motor speed가 증가할수록 반대 방향의 induced voltage가 커지고,
그 결과 armature current 증가가 억제된다.

---

## 3. Mechanical Dynamics

Motor torque는 armature current에 비례한다고 둔다.

```math
T_m(t)=K_ti(t)
```

회전 운동방정식은 다음과 같다.

```math
J_m\frac{d\omega(t)}{dt}
=
K_ti(t)
-b\omega(t)
-T_L(t)
```

따라서 angular acceleration은:

```math
\frac{d\omega(t)}{dt}
=
\frac{K_t}{J_m}i(t)
-\frac{b}{J_m}\omega(t)
-\frac{1}{J_m}T_L(t)
```

---

## 4. State-Space Model

State vector를 다음과 같이 정의한다.

```math
x(t)
=
\begin{bmatrix}
i(t)\\
\omega(t)
\end{bmatrix}
```

그러면 DC motor dynamics는 다음 `State-Space Model`로 표현할 수 있다.

```math
\dot{x}=Ax+BV+ET_L
```

System matrix:

```math
A=
\begin{bmatrix}
-\frac{R}{L} & -\frac{K_e}{L}\\
\frac{K_t}{J_m} & -\frac{b}{J_m}
\end{bmatrix}
```

Input matrix:

```math
B=
\begin{bmatrix}
\frac{1}{L}\\
0
\end{bmatrix}
```

Load disturbance matrix:

```math
E=
\begin{bmatrix}
0\\
-\frac{1}{J_m}
\end{bmatrix}
```

Output equation은 다음과 같다.

```math
y=Cx
```

Motor speed를 output으로 사용하므로:

```math
C=
\begin{bmatrix}
0 & 1
\end{bmatrix}
```

따라서:

```math
y(t)=\omega(t)
```

---

## 5. Parameter Definitions

Nominal parameter set은 literature-based value로 지정한다.

| Parameter | Meaning | Unit | v1 Value |
|---|---|---:|---:|
| $R$ | Armature resistance | $\Omega$ | `0.18644` |
| $L$ | Armature inductance | H | `0.0063` |
| $J_m$ | Rotor inertia | kg m$^2$ | `0.013767` |
| $b$ | Viscous friction coefficient | N m s/rad | `0.049813` |
| $K_t$ | Torque constant | N m/A | `0.020375` |
| $K_e$ | Back-EMF constant | V s/rad | `0.020375` |
| $V_{\max}$ | Maximum voltage | V | `12.0` |
| $T_L$ | Load torque | N m | `0` in nominal simulation |

Source:

- G. Yüksek et al., **"Enhanced DC motor speed regulation using two-degree-of-freedom PID controller tuned by animated oat optimization with simulation and real-time experimental validation"**, *Measurement and Control*, DOI: `10.1177/00202940261442256`.
- Parameter values are taken from `Table 1. DC motor parameters`.
- The paper's $J$ is mapped to this project's $J_m$.

> **Notation**
>
> Motor inertia는 $J_m$으로 표기한다.  
> Cost function은 $\mathcal{J}$로 표기하여 inertia $J_m$과 구분한다.

---

## 6. Model Assumptions

v1 simulation에서는 다음 assumptions를 사용한다.

| Assumption | Meaning |
|---|---|
| Linear motor model | $R$, $L$, $J_m$, $b$, $K_t$, $K_e$가 일정하다고 가정 |
| Armature-controlled DC motor | Input은 armature voltage $V(t)$ |
| Speed control | Output은 angular speed $\omega(t)$ |
| No load in nominal simulation | 초기 조건에서는 $T_L=0$ |
| Voltage saturation exists | 실제 controller output은 $\pm V_{\max}$로 제한 |

Nominal simulation에서는 먼저:

```math
T_L=0
```

으로 시작한다.

이후 [Robustness Test](../experiments/robustness_test.md)에서:

```math
T_L\neq0
```

조건을 추가하여 disturbance에 대한 controller 성능을 확인한다.
