
# AI-Based Motor Control Parameter Optimization

> **Project Plan v1**  
> DC Motor Speed Control을 시작으로 PID Optimization, Reinforcement Learning,
> Robustness Test, 실제 Hardware/Robot 제어까지 확장하는 프로젝트.

---

## 0. Project Pipeline

전체 프로젝트 흐름은 다음과 같다.

**Motor Modeling**  
→ **PID Baseline**  
→ **Cost Function Design**  
→ **Classical Optimization**  
→ **RL Control**  
→ **Performance Comparison**  
→ **Robustness Test**  
→ **Hardware / Robot Extension**

핵심 흐름만 단순화하면:

```text
Physical System
      ↓
Motor Model
      ↓
Controller
      ↓
Optimization / Learning
      ↓
Performance Evaluation
      ↓
Real System Extension
```

---

# 1. Motor Modeling

## 1.1 Goal

먼저 실제 `DC Motor`의 전기적·기계적 동작을 수학식으로 표현한다.

목표는 실제 모터가 없어도 Python simulation에서

```text
Input Voltage V(t)
        ↓
DC Motor Model
        ↓
Current i(t)
Angular Speed ω(t)
```

와 같이 입력 전압에 따른 motor response를 계산할 수 있도록 만드는 것이다.

현재 프로젝트의 제어 대상은 **Motor Speed**로 설정한다.

- `Input` : $V(t)$ — Motor Voltage
- `State 1` : $i(t)$ — Armature Current
- `State 2` : $\omega(t)$ — Angular Velocity
- `Output` : $\omega(t)$ — Motor Speed
- `Disturbance` : $T_L(t)$ — Load Torque

> **핵심**
>
> Motor Modeling은 실제 모터를 Python 안에서 동작시킬 수 있도록  
> **수학적인 가상 모터를 만드는 과정**이라고 생각하면 된다.

---

## 1.2 Electrical Dynamics

DC motor의 armature 회로에 `KVL`을 적용하면

```math
V(t)
=
L\frac{di(t)}{dt}
+
Ri(t)
+
K_e\omega(t)
```
이다.

이를 current derivative 형태로 정리하면

```math
\frac{di(t)}{dt}
=
-\frac{R}{L}i(t)
-\frac{K_e}{L}\omega(t)
+\frac{1}{L}V(t)
```
이다.

### Parameters

- `R` : Armature Resistance
- `L` : Armature Inductance
- `K_e` : Back-EMF Constant
- `V(t)` : Applied Voltage
- `i(t)` : Armature Current
- `ω(t)` : Angular Velocity

모터가 회전하면 `Back EMF`가 발생한다.

```math
e_b(t)=K_e\omega(t)
```
즉 motor speed가 증가할수록 반대 방향의 induced voltage가 증가하여
current 증가를 억제한다.

---

## 1.3 Mechanical Dynamics

Motor Torque는 armature current에 비례한다고 가정한다.

```math
T_m(t)=K_ti(t)
```
회전 운동방정식은

```math
J_m\frac{d\omega(t)}{dt}
=
K_ti(t)
-b\omega(t)
-T_L(t)
```
로 둔다.

따라서

```math
\frac{d\omega(t)}{dt}
=
\frac{K_t}{J_m}i(t)
-\frac{b}{J_m}\omega(t)
-\frac{1}{J_m}T_L(t)
```
이다.

### Parameters

- `J_m` : Rotor Inertia
- `b` : Viscous Friction Coefficient
- `K_t` : Torque Constant
- `T_L` : External Load Torque

> **Notation**
>
> Motor inertia는 $J_m$으로 표기한다.
> Cost Function은 이후 $\mathcal{J}$로 표기하여 둘을 구분한다.

---

## 1.4 State-Space Model

State vector를

```math
x(t)
=
\begin{bmatrix}
i(t)\\
\omega(t)
\end{bmatrix}
```
로 정의한다.

그러면 DC motor dynamics는

```math
\boxed{
\dot{x}
=
Ax+BV+ET_L
}
```
형태의 `State-Space Model`로 표현할 수 있다.

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
Output equation:

```math
y=Cx
```
이며 motor speed를 output으로 사용하므로

```math
C=
\begin{bmatrix}
0&1
\end{bmatrix}
```
이다.

따라서

```math
\boxed{
y(t)=\omega(t)
}
```
가 된다.

---

## 1.5 v1 Assumption

초기 simulation에서는 외부 부하가 없는 조건으로 시작한다.

```math
\boxed{
T_L=0
}
```
이후 `Robustness Test`에서

```math
T_L\neq0
```
조건을 추가하여 load disturbance에 대한 controller 성능을 확인한다.

---

# 2. PID Baseline

## 2.1 Why PID?

첫 번째 controller는 가장 기본적이고 널리 사용되는
`PID Controller`로 설정한다.

목표 속도를 $\omega_{\mathrm{ref}}(t)$, 실제 motor speed를 $\omega(t)$라고 하면 Tracking Error는

```math
\boxed{
e(t)=\omega_{\mathrm{ref}}(t)-\omega(t)
}
```
이다.

PID Control Law는

```math
\boxed{
V(t)
=
K_pe(t)
+
K_i\int e(t)\,dt
+
K_d\frac{de(t)}{dt}
}
```
로 정의한다.

여기서 PID controller의 output $V(t)$는
DC motor에 실제로 입력되는 voltage command가 된다.

---

## 2.2 PID Gains

PID에서 조정해야 하는 세 parameter는

```text
Kp
Ki
Kd
```

이다.

이 세 값을 `PID Gains`라고 한다.

### Kp — Proportional Gain

현재 error의 크기에 비례해서 반응한다.

```text
현재 Error 큼
      ↓
Kp × Error 큼
      ↓
강한 Control Input
```

즉 목표 speed와 실제 speed 차이가 클수록
더 강하게 motor를 가속시키는 역할을 한다.

단, `Kp`가 지나치게 크면 overshoot 또는 oscillation이 커질 수 있다.

---

### Ki — Integral Gain

과거부터 누적된 error를 이용한다.

```math
K_i\int e(t)\,dt
```
작은 error가 오랫동안 남아 있다면 integral term이 계속 누적된다.

따라서 `Ki`는 주로

`Steady-State Error`

를 제거하는 역할을 한다.

---

### Kd — Derivative Gain

현재 error가 얼마나 빠르게 변하는지를 본다.

```math
K_d\frac{de(t)}{dt}
```
속도가 너무 빠르게 목표값을 넘어가려고 할 경우
변화율을 이용해 response를 완화하는 역할을 할 수 있다.

즉 간단히 말하면

```text
Kp = 지금 Error에 반응
Ki = 과거 Error를 누적
Kd = Error 변화 속도에 반응
```

이다.

---

## 2.3 Baseline Controller

처음부터 optimization을 사용하지 않고
먼저 기본 PID gain을 직접 설정한다.

이를

`Manual PID`

또는

`PID Baseline`

으로 사용한다.

이후 결과를

```text
Manual PID
    ↓
Optimized PID
    ↓
RL Controller
```

순서로 비교한다.

PID Baseline의 목적은 단순히 controller를 만드는 것이 아니라
이후 optimization과 RL의 성능을 비교할 **기준점**을 만드는 것이다.

---

# 3. Cost Function Design

## 3.1 Why Cost Function?

Optimization algorithm은 motor response graph를 보고

> "이 그래프가 더 좋아 보인다."

라고 판단할 수 없다.

따라서 **좋은 제어가 무엇인지 하나의 numerical score로 표현**해야 한다.

그 값을 본 프로젝트에서는 $\mathcal{J}$로 표시한다.

정의는 다음과 같다.

```math
\boxed{
\mathcal{J}\text{가 작을수록 좋은 Controller}
}
```
따라서 PID optimization problem은

```math
\boxed{
\min_{K_p,K_i,K_d}
\mathcal{J}(K_p,K_i,K_d)
}
```
이다.

---

## 3.2 What is Good Control?

v1에서는 좋은 controller를 세 가지 기준으로 평가한다.

1. 목표 speed를 잘 따라가야 한다.  
   `Tracking Performance`

2. 목표 speed를 너무 많이 넘어가면 안 된다.  
   `Overshoot`

3. 지나치게 큰 voltage를 계속 사용하면 안 된다.  
   `Control Effort`

따라서 Cost Function을

```math
\boxed{
\mathcal{J}
=
w_e\mathcal{J}_{tracking}
+
w_o\mathcal{J}_{overshoot}
+
w_u\mathcal{J}_{control}
}
```
형태로 구성한다.

---

## 3.3 Tracking Error Cost

Tracking Error:

```math
e(t)
=
\omega_{\mathrm{ref}}(t)-\omega(t)
```
가장 기본적인 error metric으로 `ISE`가 있다.

```math
ISE
=
\int_0^T e^2(t)\,dt
```
Error를 제곱하기 때문에 positive error와 negative error가
서로 상쇄되지 않는다.

또한 큰 error에 더 큰 penalty를 준다.

---

### ISE → ITSE

단순 ISE에서는 simulation 초기의 error와
오랜 시간이 지난 뒤의 error를 동일한 방식으로 평가한다.

하지만 motor가 처음 출발할 때 발생하는 큰 error보다

> 시간이 많이 지났는데도 error가 계속 남아 있는 상황

을 더 나쁘게 평가하고 싶다.

따라서 시간 $t$를 곱한 `ITSE`를 사용한다.

```math
ITSE
=
\int_0^T te^2(t)\,dt
```
즉 시간이 지날수록 남아 있는 error에 더 큰 penalty가 적용된다.

```text
Early Error
   → 상대적으로 작은 penalty

Late Error
   → 큰 penalty
```

---

## 3.4 Normalized Tracking Error

Reference speed 자체가 달라지면 error의 절대값도 달라진다.

예를 들어

```text
100 rpm 기준의 10 rpm error
1000 rpm 기준의 10 rpm error
```

는 의미가 다를 수 있다.

따라서 normalized error를 정의한다.

```math
\boxed{
\tilde e(t)
=
\frac{
\omega_{\mathrm{ref}}-\omega(t)
}{
\omega_{\mathrm{ref}}
}
}
```
그리고 Tracking Cost v1은

```math
\boxed{
\mathcal{J}_{tracking}
=
\frac{1}{T^2}
\int_0^T
t\tilde e^2(t)\,dt
}
```
로 정의한다.

---

## 3.5 Overshoot Cost

예를 들어 목표 speed가

```text
1000 rpm
```

인데 response가 순간적으로

```text
1400 rpm
```

까지 올라간 후 1000 rpm으로 내려왔다고 하자.

최종적으로 reference에 도달했더라도
좋은 controller라고 보기 어렵다.

Normalized Overshoot:

```math
\boxed{
M_p
=
\frac{
\omega_{\max}-\omega_{\mathrm{ref}}
}{
\omega_{\mathrm{ref}}
}
}
```
예를 들어

```math
\omega_{\mathrm{ref}}=1000
```
```math
\omega_{\max}=1400
```
이면

```math
M_p
=
\frac{1400-1000}{1000}
=
0.4
```
즉

```text
40 % Overshoot
```

이다.

v1에서는

```math
\boxed{
\mathcal{J}_{overshoot}=M_p^2
}
```
로 사용한다.

Overshoot가 발생하지 않았다면

```math
\boxed{
\mathcal{J}_{overshoot}=0
}
```
으로 둔다.

---

## 3.6 Control Effort Cost

Tracking Error만 평가하면 optimizer는
아주 큰 voltage를 사용해서 motor를 빠르게 reference에 도달시키는
solution을 선택할 수 있다.

예를 들어

```text
Tracking = 매우 좋음
Voltage  = 계속 Maximum
```

이라면 실제 system에서는 좋은 controller라고 하기 어렵다.

따라서 control input에도 penalty를 준다.

Normalized Control Input:

```math
\boxed{
\tilde u(t)
=
\frac{V(t)}{V_{\max}}
}
```
Control Effort Cost:

```math
\boxed{
\mathcal{J}_{control}
=
\frac{1}{T}
\int_0^T
\tilde u^2(t)\,dt
}
```
즉 큰 voltage를 오랫동안 사용하는 controller는
더 높은 cost를 받는다.

---

## 3.7 Cost Function v1

현재 v1에서는 다음 weighting을 initial setting으로 사용한다.

| Cost Term | Weight | Meaning |
|---|---:|---|
| `Tracking Error` | 0.60 | 목표 speed를 얼마나 잘 따라가는가 |
| `Overshoot` | 0.25 | 목표 speed를 얼마나 많이 넘어가는가 |
| `Control Effort` | 0.15 | 과도한 voltage를 사용하는가 |

따라서 최종 Cost Function v1은

```math
\boxed{
\mathcal{J}
=
0.60\mathcal{J}_{tracking}
+
0.25\mathcal{J}_{overshoot}
+
0.15\mathcal{J}_{control}
}
```
이다.

> **Important**
>
> `0.60 / 0.25 / 0.15`는 아직 최종적으로 검증된 값이 아니다.  
> 현재는 v1 initial design으로 사용하고,
> 이후 simulation 결과에 따라 weighting sensitivity를 분석할 수 있다.

---

# 4. Classical Optimization

## 4.1 Optimization Target

Optimizer가 찾는 parameter는 PID Gains이다.

```math
\theta
=
\begin{bmatrix}
K_p\\
K_i\\
K_d
\end{bmatrix}
```
목적은

```math
\boxed{
\theta^*
=
\arg\min_{\theta}
\mathcal{J}(\theta)
}
```
이다.

즉 최종적으로 얻고 싶은 값은

```math
\boxed{
K_p^*,\quad K_i^*,\quad K_d^*
}
```
이다.

---

## 4.2 Optimization Loop

전체 optimization 구조는 다음과 같다.

```text
Optimizer
    ↓
(Kp, Ki, Kd)
    ↓
PID Controller
    ↓
Motor Simulation
    ↓
ω(t), V(t), e(t)
    ↓
Cost Function
    ↓
𝓙
    ↓
Optimizer
```

예를 들어 optimizer가

```text
Kp = 2.1
Ki = 0.8
Kd = 0.1
```

을 candidate로 선택했다고 하자.

PID controller가 이 gain을 사용해 motor simulation을 수행한다.

Simulation 결과로

```text
ω(t)
V(t)
e(t)
```

를 얻는다.

이를 이용해

```text
Tracking Cost
Overshoot Cost
Control Effort Cost
```

를 계산하고 최종적으로

```text
𝓙 = 0.43
```

과 같은 하나의 값을 optimizer에 반환한다.

Optimizer는 이 결과를 기반으로 다음

```text
(Kp, Ki, Kd)
```

candidate를 선택한다.

이 과정을 반복하여 $\mathcal{J}$가 작은 PID Gains를 찾는다.

---

## 4.3 Current Optimizers

v1에서 비교할 Classical Optimization 방법은 다음과 같다.

### Random Search

PID gain search range 안에서
random한 candidate를 생성한다.

구현이 단순하므로 가장 기본적인 optimization baseline으로 사용한다.

---

### Genetic Algorithm

여러 PID candidate를 population으로 관리한다.

```text
Population
    ↓
Selection
    ↓
Crossover
    ↓
Mutation
    ↓
New Population
```

Cost가 작은 candidate가 다음 generation에 살아남도록 한다.

---

### Bayesian Optimization

지금까지 평가한

```math
(K_p,K_i,K_d)
\rightarrow
\mathcal{J}
```
관계를 이용해 surrogate model을 만든다.

그 후

> 다음에는 어느 parameter를 평가하는 것이 유리한가?

를 판단하면서 search한다.

---

### Optimization Order

초기 실험 순서는 다음과 같이 진행한다.

```text
Manual PID
    ↓
Random Search
    ↓
Genetic Algorithm
    ↓
Bayesian Optimization
```

---

# 5. RL Control

## 5.1 Difference from PID Optimization

Classical Optimization에서는

```text
Kp
Ki
Kd
```

라는 **고정된 PID parameter**를 찾는다.

반면 `Reinforcement Learning`에서는
Agent가 현재 motor state를 관찰하고
매 time step마다 직접 control action을 결정하도록 한다.

즉:

```text
PID Optimization
→ 좋은 고정 PID Gains 찾기

RL Control
→ 상태를 보고 매 순간 Action 결정하기
```

라는 차이가 있다.

---

## 5.2 State

Possible RL State v1:

```math
\boxed{
s_t
=
\begin{bmatrix}
e_t\\
\omega_t\\
i_t
\end{bmatrix}
}
```
즉 Agent가 보는 정보는

- Current Tracking Error
- Motor Speed
- Motor Current

이다.

---

## 5.3 Action

RL Agent가 결정하는 action은 motor voltage로 설정하는 것을
우선 고려한다.

```math
\boxed{
a_t=V_t
}
```
실제 hardware에서는 이후

`PWM Duty`

형태로 변경할 수 있다.

---

## 5.4 Policy

RL Agent는 현재 state를 입력받아 action을 결정한다.

```math
\boxed{
s_t
\rightarrow
\pi_\theta
\rightarrow
a_t
}
```
전체 interaction은

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

형태가 된다.

---

## 5.5 Reward Function

PID Optimization에서는 $\mathcal{J}\downarrow$를 목표로 한다.

RL에서는 반대로 $\mathrm{Reward}\uparrow$를 목표로 한다.

Possible Reward v1:

```math
r_t
=
-\alpha\tilde e_t^2
-\beta\tilde u_t^2
```
즉

```text
Tracking Error 큼
→ Reward 감소

Control Input 너무 큼
→ Reward 감소
```

하도록 설계한다.

따라서

```text
PID Optimization → Cost 최소화
RL Control       → Reward 최대화
```

이지만 두 방법이 원하는 물리적 목적은 동일하다.

> **Good Motor Control**

---

# 6. Performance Comparison

최종적으로 다음 세 controller를 비교한다.

```math
\boxed{
\text{Manual PID}
\quad vs \quad
\text{Optimized PID}
\quad vs \quad
\text{RL Controller}
}
```
---

## 6.1 Evaluation Metrics

비교할 주요 metric:

- `Tracking Error`
- `Overshoot`
- `Settling Time`
- `Steady-State Error`
- `Control Effort`
- `Total Cost`
- `Robustness`

---

## 6.2 Result Table

최종적으로 아래와 같은 table을 만드는 것이 목표다.

| Controller | Tracking Error | Overshoot | Settling Time | Control Effort | Total Cost |
|---|---:|---:|---:|---:|---:|
| Manual PID | TBD | TBD | TBD | TBD | TBD |
| Optimized PID | TBD | TBD | TBD | TBD | TBD |
| RL Controller | TBD | TBD | TBD | TBD | TBD |

그래프에서는 각 controller의 $\omega(t)$ response를 동일한 reference input에 대해 비교한다.

---

# 7. Robustness Test

## 7.1 Goal

Nominal condition에서 좋은 controller가
실제 환경 변화에서도 잘 작동하는지 확인한다.

실제 motor에서는 parameter가 정확하게 일정하지 않을 수 있다.

원인 예시:

- Motor Temperature
- Load Change
- Mechanical Friction
- Parameter Uncertainty
- Sensor Noise

따라서 nominal model을 일부 변경하면서 controller 성능을 다시 측정한다.

---

## 7.2 Resistance Variation

Motor temperature가 변화하면 armature resistance가 변할 수 있다.

예:

```math
\boxed{
R\rightarrow1.2R
}
```
---

## 7.3 Inertia Variation

Motor shaft에 추가 load가 연결되면
effective inertia가 증가할 수 있다.

예:

```math
\boxed{
J_m\rightarrow1.3J_m
}
```
---

## 7.4 Load Disturbance

Nominal simulation에서는

```math
T_L=0
```
으로 시작하지만 robustness test에서는

```math
\boxed{
T_L\neq0
}
```
을 적용한다.

예를 들어 motor가 일정 speed로 회전하는 도중
갑자기 load torque를 적용할 수 있다.

```text
Normal Operation
      ↓
Sudden Load Torque
      ↓
Speed Drop
      ↓
Controller Recovery
```

이때 얼마나 빠르게 reference speed로 복귀하는지 확인한다.

---

## 7.5 Sensor Noise

실제 encoder measurement에는 noise가 존재할 수 있다.

따라서 measured speed를

```math
\omega_{\mathrm{measured}}
=
\omega+\text{noise}
```
형태로 구성하여 controller 성능을 확인한다.

---

## 7.6 Main Question

Robustness Test에서 확인하고 싶은 핵심 질문은

> **Nominal Motor Model에서 찾은 Controller가  
> Motor Parameter와 외란이 변해도 안정적으로 동작하는가?**

이다.

---

# 8. Hardware / Robot Extension

Simulation 단계가 충분히 검증된 이후
실제 motor hardware로 확장한다.

Possible Hardware:

```text
STM32 / Arduino
Motor Driver
DC Motor
Encoder
Current Sensor
Power Supply
```

전체 physical control loop:

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

Simulation에서 사용했던 controller를 실제 embedded system으로
옮기는 것이 첫 번째 hardware 목표다.

---

## 8.1 Mobile Robot Extension

DC Motor 하나의 speed control이 성공하면
두 개의 motor를 사용하는 `Differential Drive Robot`으로 확장한다.

Left Wheel Speed: $\omega_L$

Right Wheel Speed: $\omega_R$

이를 이용하면

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

으로 확장할 수 있다.

최종적으로는 단순 motor control이 아니라

```math
\boxed{
\text{Trajectory Tracking}
}
```
문제로 발전시키는 것을 long-term extension으로 둔다.

---

# 9. Optional Quantum Extension

Quantum Computing은 본 프로젝트의 Main Pipeline에서는 분리한다.

핵심 프로젝트는

```text
Motor Modeling
      ↓
PID
      ↓
Classical Optimization
      ↓
Reinforcement Learning
```

만으로 완성되어야 한다.

Quantum Optimization은 이후 optional experiment로 추가한다.

Possible Pipeline:

```text
Continuous PID Gains
        ↓
Discretization
        ↓
Binary Encoding
        ↓
QUBO Formulation
        ↓
QAOA
        ↓
Warm-start QAOA
        ↓
Classical Optimizer Comparison
```

Continuous variable인 $K_p$, $K_i$, $K_d$를 binary representation으로 바꾼 후
QUBO 형태로 변환하는 방법을 검토한다.

> **Principle**
>
> QAOA가 구현되지 않아도 Main Project는 완성되어야 한다.  
> Quantum Optimization은 추가 연구 및 비교 실험으로 취급한다.

---

# 10. Current v1 Decisions

현재 `Project v1`에서 확정한 사항을 정리한다.

```text
Target System
└─ Armature-Controlled DC Motor

Control Target
└─ Angular Speed ω(t)

Control Input
└─ Voltage V(t)

System States
├─ Armature Current i(t)
└─ Angular Speed ω(t)

Disturbance
└─ Load Torque T_L(t)

Baseline Controller
└─ PID Controller

Optimization Variables
├─ Kp
├─ Ki
└─ Kd

Optimization Objective
└─ minimize 𝓙(Kp, Ki, Kd)

Cost Function
├─ Tracking Error
├─ Overshoot
└─ Control Effort

Classical Optimization
├─ Random Search
├─ Genetic Algorithm
└─ Bayesian Optimization

Learning-Based Control
└─ Reinforcement Learning

Robustness Test
├─ Resistance Variation
├─ Inertia Variation
├─ Load Disturbance
└─ Sensor Noise

Hardware Extension
└─ DC Motor Hardware

Robot Extension
└─ Differential Drive / Trajectory Tracking

Quantum Extension
└─ Optional
```

---

# 11. Cost Function v1 Summary

현재 Cost Function은

```math
\boxed{
\mathcal{J}
=
0.60\mathcal{J}_{tracking}
+
0.25\mathcal{J}_{overshoot}
+
0.15\mathcal{J}_{control}
}
```
로 시작한다.

각 항은 다음과 같다.

### Tracking

```math
\mathcal{J}_{tracking}
=
\frac{1}{T^2}
\int_0^T
t
\left(
\frac{
\omega_{\mathrm{ref}}-\omega(t)
}{
\omega_{\mathrm{ref}}
}
\right)^2dt
```
### Overshoot

```math
\mathcal{J}_{overshoot}
=
M_p^2
```
```math
M_p
=
\max
\left(
0,
\frac{
\omega_{\max}-\omega_{\mathrm{ref}}
}{
\omega_{\mathrm{ref}}
}
\right)
```
### Control Effort

```math
\mathcal{J}_{control}
=
\frac{1}{T}
\int_0^T
\left(
\frac{V(t)}{V_{\max}}
\right)^2dt
```
따라서 optimization의 핵심은

```math
\boxed{
(K_p^*,K_i^*,K_d^*)
=
\arg\min_{K_p,K_i,K_d}
\mathcal{J}(K_p,K_i,K_d)
}
```
이다.

---

# 12. Parameters To Be Determined

Motor model의 구조는 확정했지만
아직 numerical parameter는 결정하지 않았다.

| Parameter | Meaning | Status |
|---|---|---|
| `R` | Armature Resistance | TBD |
| `L` | Armature Inductance | TBD |
| `J_m` | Rotor Inertia | TBD |
| `b` | Viscous Friction | TBD |
| `K_t` | Torque Constant | TBD |
| `K_e` | Back-EMF Constant | TBD |
| `V_max` | Maximum Motor Voltage | TBD |
| `ω_ref` | Reference Speed | TBD |
| `T` | Simulation Time | TBD |
| `Kp range` | Proportional Gain Search Range | TBD |
| `Ki range` | Integral Gain Search Range | TBD |
| `Kd range` | Derivative Gain Search Range | TBD |

이 값들은 실제 simulation을 시작하기 전에 결정한다.

---

# 13. Development Strategy

프로젝트는 처음부터 모든 것을 한꺼번에 구현하지 않는다.

단계별로 동작을 확인한다.

```text
Step 1
DC Motor Parameter 결정
        ↓
Step 2
Motor Model 구현
        ↓
Step 3
Open-Loop Simulation
        ↓
Step 4
PID Baseline 구현
        ↓
Step 5
Closed-Loop Response 확인
        ↓
Step 6
Cost Function 구현
        ↓
Step 7
Random Search
        ↓
Step 8
Genetic Algorithm
        ↓
Step 9
Bayesian Optimization
        ↓
Step 10
Performance Comparison
        ↓
Step 11
RL Controller
        ↓
Step 12
Robustness Test
        ↓
Step 13
Hardware / Robot Extension
```

각 단계가 정상적으로 동작한 것을 확인한 뒤
다음 단계로 진행한다.

---

# 14. Next Step

현재 `Project Plan v1`에서는 다음 사항까지 결정되었다.

```text
✓ Control Target
✓ Motor Model Structure
✓ State Variables
✓ PID Baseline Structure
✓ Optimization Variables
✓ Cost Function Structure
✓ Classical Optimizer Candidates
✓ RL Extension Direction
✓ Robustness Test Direction
✓ Hardware / Robot Extension Direction
```

다음 단계에서 가장 먼저 결정해야 할 것은
**Nominal DC Motor Parameters**이다.

즉 다음 값을 정해야 한다.

```math
R,\quad
L,\quad
J_m,\quad
b,\quad
K_t,\quad
K_e
```
그 후 첫 번째 실제 implementation은

```text
DC Motor Parameters
        ↓
Python Motor Model
        ↓
Open-Loop Step Response
```

순서로 진행한다.

> **Next Question**
>
> 어떤 DC Motor Parameter Set을 기준 모델로 사용할 것인가?
