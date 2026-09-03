# Cost Function Design

이 문서는 PID optimization과 controller 비교에 사용할 cost function을 정리한다.

Optimizer는 response graph를 보고 "좋다" 또는 "나쁘다"라고 판단할 수 없다.
따라서 좋은 control response를 하나의 numerical score로 바꿔야 한다.

본 프로젝트에서는 그 값을 $\mathcal{J}$로 표기한다.

```math
\mathcal{J}\downarrow
```

$\mathcal{J}$가 작을수록 좋은 controller로 판단한다.
---

## 1. What is Good Control?

v1에서는 좋은 controller를 세 가지 기준으로 평가한다.

| Criterion | Meaning |
|---|---|
| Tracking Performance | 목표 speed를 잘 따라가는가 |
| Overshoot | 목표 speed를 너무 많이 넘어가지 않는가 |
| Control Effort | 지나치게 큰 voltage를 계속 사용하지 않는가 |

따라서 cost function은 다음 형태로 구성한다.

```math
\mathcal{J}
=
w_e\mathcal{J}_{tracking}
+
w_o\mathcal{J}_{overshoot}
+
w_u\mathcal{J}_{control}
```

---

## 2. Normalized Tracking Error

Tracking error는 다음과 같다.

```math
e(t)=\omega_{\mathrm{ref}}-\omega(t)
```

Reference speed가 달라지면 error의 절대값도 달라지므로,
v1에서는 normalized tracking error를 사용한다.

```math
\tilde{e}(t)
=
\frac{\omega_{\mathrm{ref}}-\omega(t)}{\omega_{\mathrm{ref}}}
```

예를 들어 `100 rpm 기준의 10 rpm error`와
`1000 rpm 기준의 10 rpm error`는 의미가 다르다.
Normalized error는 이 차이를 반영한다.

---

## 3. ITSE Tracking Cost

기본 error metric으로는 `ISE`를 사용할 수 있다.

```math
ISE=\int_0^T e^2(t)\,dt
```

하지만 단순 ISE는 simulation 초반 error와 후반 error를 비슷하게 취급한다.
Motor가 처음 출발할 때 생기는 error보다, 시간이 많이 지났는데도 남아 있는 error를
더 나쁘게 보고 싶다면 `ITSE`가 더 적절하다.

```math
ITSE=\int_0^T t e^2(t)\,dt
```

v1 tracking cost는 normalized error를 사용한 ITSE 형태로 둔다.

```math
\mathcal{J}_{tracking}
=
\frac{1}{T^2}
\int_0^T
t\tilde{e}^{\,2}(t)\,dt
```

```text
Early Error
   → 상대적으로 작은 penalty

Late Error
   → 더 큰 penalty
```

---

## 4. Overshoot Cost

목표 speed가 `1000 rpm`인데 response가 순간적으로 `1400 rpm`까지 올라간 뒤
다시 내려온다면, 최종적으로 reference에 도달하더라도 좋은 response라고 보기 어렵다.

Normalized overshoot는 다음과 같이 둔다.

```math
M_p
=
\max
\left(
0,
\frac{\omega_{\max}-\omega_{\mathrm{ref}}}{\omega_{\mathrm{ref}}}
\right)
```

Overshoot cost는:

```math
\mathcal{J}_{overshoot}=M_p^2
```

Overshoot가 발생하지 않으면 $M_p=0$이고,
따라서 $\mathcal{J}_{overshoot}=0$이다.

---

## 5. Control-Effort Cost

Tracking error만 평가하면 optimizer는 아주 큰 voltage를 사용해서
motor를 빠르게 reference에 도달시키는 solution을 선택할 수 있다.

예를 들어:

```text
Tracking = 매우 좋음
Voltage  = 계속 Maximum
```

이라면 실제 system에서는 좋은 controller라고 보기 어렵다.

따라서 normalized control input을 정의한다.

```math
\tilde{u}(t)=\frac{V(t)}{V_{\max}}
```

Control-effort cost는 다음과 같다.

```math
\mathcal{J}_{control}
=
\frac{1}{T}
\int_0^T
\tilde{u}^{\,2}(t)\,dt
```

큰 voltage를 오랫동안 사용하는 controller는 더 높은 cost를 받는다.

---

## 6. Weighted Cost Function v1

현재 v1에서는 다음 weighting을 initial design choice로 사용한다.

| Cost Term | Weight | Meaning |
|---|---:|---|
| $\mathcal{J}_{tracking}$ | 0.60 | 목표 speed를 얼마나 잘 따라가는가 |
| $\mathcal{J}_{overshoot}$ | 0.25 | 목표 speed를 얼마나 많이 넘어가는가 |
| $\mathcal{J}_{control}$ | 0.15 | 과도한 voltage를 사용하는가 |

최종 cost function v1은 다음과 같다.

```math
\mathcal{J}
=
0.60\mathcal{J}_{tracking}
+
0.25\mathcal{J}_{overshoot}
+
0.15\mathcal{J}_{control}
```

> **Important**
>
> `0.60 / 0.25 / 0.15`는 v1 design choice이다.  
> Optimizer가 찾는 변수는 이 weight들이 아니라 $K_p$, $K_i$, $K_d$이다.

---

## 7. Role in Optimization

PID optimization의 목적은 다음과 같다.

```math
\min_{K_p,K_i,K_d}\mathcal{J}(K_p,K_i,K_d)
```

즉 optimizer는 여러 PID gains candidate를 simulation에 넣고,
각 candidate의 response에서 $\mathcal{J}$를 계산한 뒤,
더 작은 cost를 만드는 gains를 찾는다.

Baseline PID의 cost $\mathcal{J}_{baseline}$은 optimizer를 실행하기 위한 필수 입력이 아니라,
optimization 이후 성능을 비교하기 위한 benchmark로 사용한다.
