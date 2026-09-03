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

$$
V(t)
=
L\frac{di(t)}{dt}
+
Ri(t)
+
K_e\omega(t)
$$

이를 current derivative 형태로 정리하면:

$$
\frac{di(t)}{dt}
=
-\frac{R}{L}i(t)
-\frac{K_e}{L}\omega(t)
+\frac{1}{L}V(t)
$$

여기서 $K_e\omega(t)$는 `Back EMF`이다.

$$
e_b(t)=K_e\omega(t)
$$

Motor speed가 증가할수록 반대 방향의 induced voltage가 커지고,
그 결과 armature current 증가가 억제된다.

---

## 3. Mechanical Dynamics

Motor torque는 armature current에 비례한다고 둔다.

$$
T_m(t)=K_ti(t)
$$

회전 운동방정식은 다음과 같다.

$$
J_m\frac{d\omega(t)}{dt}
=
K_ti(t)
-b\omega(t)
-T_L(t)
$$

따라서 angular acceleration은:

$$
\frac{d\omega(t)}{dt}
=
\frac{K_t}{J_m}i(t)
-\frac{b}{J_m}\omega(t)
-\frac{1}{J_m}T_L(t)
$$

---

## 4. State-Space Model

State vector를 다음과 같이 정의한다.

$$
x(t)
=
\begin{bmatrix}
i(t)\\
\omega(t)
\end{bmatrix}
$$

그러면 DC motor dynamics는 다음 `State-Space Model`로 표현할 수 있다.

$$
\dot{x}=Ax+BV+ET_L
$$

System matrix:

$$
A=
\begin{bmatrix}
-\frac{R}{L} & -\frac{K_e}{L}\\
\frac{K_t}{J_m} & -\frac{b}{J_m}
\end{bmatrix}
$$

Input matrix:

$$
B=
\begin{bmatrix}
\frac{1}{L}\\
0
\end{bmatrix}
$$

Load disturbance matrix:

$$
E=
\begin{bmatrix}
0\\
-\frac{1}{J_m}
\end{bmatrix}
$$

Output equation은 다음과 같다.

$$
y=Cx
$$

Motor speed를 output으로 사용하므로:

$$
C=
\begin{bmatrix}
0 & 1
\end{bmatrix}
$$

따라서:

$$
y(t)=\omega(t)
$$

---

## 5. Parameter Definitions

| Parameter | Meaning | Unit | Status |
|---|---|---:|---|
| $R$ | Armature resistance | $\Omega$ | TBD |
| $L$ | Armature inductance | H | TBD |
| $J_m$ | Rotor inertia | kg m$^2$ | TBD |
| $b$ | Viscous friction coefficient | N m s/rad | TBD |
| $K_t$ | Torque constant | N m/A | TBD |
| $K_e$ | Back-EMF constant | V s/rad | TBD |
| $V_{\max}$ | Maximum voltage | V | TBD |
| $T_L$ | Load torque | N m | TBD |

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

$$
T_L=0
$$

으로 시작한다.

이후 [Robustness Test](../experiments/robustness_test.md)에서:

$$
T_L\neq0
$$

조건을 추가하여 disturbance에 대한 controller 성능을 확인한다.
