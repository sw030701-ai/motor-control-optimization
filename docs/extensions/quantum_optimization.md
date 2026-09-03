# Optional Quantum Optimization

이 문서는 quantum optimization을 optional extension으로 정리한다.

> **Principle**
>
> Quantum optimization이 구현되지 않아도 main project는 완성되어야 한다.  
> Core project는 `Motor Modeling → PID → Classical Optimization → RL / Robustness`만으로 독립적으로 성립해야 한다.

---

## 1. Position in the Project

Quantum optimization은 main pipeline에 넣지 않는다.

```text
Core Project
Motor Modeling
      ↓
PID Baseline
      ↓
Classical Optimization
      ↓
RL Controller
      ↓
Robustness Test
```

Quantum optimization은 위 흐름이 완성된 이후 추가 연구 주제로 다룬다.

---

## 2. Why Optional?

PID gains는 원래 continuous variable이다.

```math
K_p,\quad K_i,\quad K_d\in\mathbb{R}
```

하지만 QUBO 또는 QAOA는 보통 binary variable 형태의 문제를 요구한다.
따라서 quantum optimization을 적용하려면 PID gain을 먼저 discretization해야 한다.

이 과정 자체가 추가적인 modeling decision이므로,
core motor control project와 분리해서 다루는 것이 적절하다.

---

## 3. Possible Pipeline

Quantum extension의 possible pipeline은 다음과 같다.

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

---

## 4. PID-Gain Discretization

먼저 각 gain range를 discrete grid로 나눈다.

```text
Kp ∈ {Kp_1, Kp_2, ..., Kp_N}
Ki ∈ {Ki_1, Ki_2, ..., Ki_N}
Kd ∈ {Kd_1, Kd_2, ..., Kd_N}
```

Continuous PID optimization 문제는 discrete search 문제로 바뀐다.

```math
\min_{K_p,K_i,K_d}\mathcal{J}(K_p,K_i,K_d)
```

---

## 5. Binary Encoding

Discrete gain index를 binary variable로 표현한다.

예를 들어 $K_p$ 후보가 8개라면 3-bit encoding을 사용할 수 있다.

```text
000 → Kp_1
001 → Kp_2
010 → Kp_3
...
111 → Kp_8
```

같은 방식으로 $K_i$, $K_d$도 binary encoding한다.

---

## 6. QUBO Formulation

QUBO는 다음 형태의 binary optimization problem이다.

```math
\min_{z\in\{0,1\}^n} z^TQz
```

PID tuning 문제를 QUBO로 바꾸려면,
binary vector $z$가 특정 PID gains combination을 의미하도록 만들고,
그 조합의 cost가 QUBO objective에 반영되도록 구성해야 한다.

이 변환은 단순하지 않으므로, core project가 완성된 뒤 별도 experiment로 설계한다.

---

## 7. QAOA and Warm-Start QAOA

QAOA는 QUBO 형태의 combinatorial optimization problem에 적용할 수 있는 quantum algorithm 후보이다.

Possible comparison:

| Method | Role |
|---|---|
| Random Search | Classical discrete search baseline |
| Bayesian Optimization | Main classical optimizer |
| QAOA | Quantum optimization candidate |
| Warm-start QAOA | Classical solution을 초기점으로 활용하는 quantum variant |

---

## 8. When to Start

Quantum extension은 다음 조건을 만족한 뒤 시작한다.

- DC motor simulation이 안정적으로 동작한다.
- Manual PID baseline이 확정되어 있다.
- Classical PID optimization 결과가 있다.
- Cost function과 evaluation metrics가 고정되어 있다.
- Discrete PID gain search space를 정의할 수 있다.

즉 quantum optimization은 core project를 대체하는 것이 아니라,
완성된 PID optimization problem을 다른 방식으로 풀어보는 optional comparison이다.
