# Classical Optimization

이 문서는 `PID Gains`를 자동으로 찾기 위한 classical optimization 구조를 정리한다.

Main story는 다음 흐름으로 둔다.

```text
Manual PID
      ↓
Random Search
      ↓
Bayesian Optimization
```

`Genetic Algorithm`은 optional comparison으로 사용한다.

---

## 1. Optimization Objective

Optimizer가 찾는 parameter는 PID gains이다.

```math
\theta
=
\begin{bmatrix}
K_p\\
K_i\\
K_d
\end{bmatrix}
```

목적은 cost function을 최소화하는 gains를 찾는 것이다.

```math
\min_{K_p,K_i,K_d}\mathcal{J}(K_p,K_i,K_d)
```

또는:

```math
\theta^*
=
\arg\min_{\theta}\mathcal{J}(\theta)
```

최종적으로 얻고 싶은 값은 다음과 같다.

```math
K_p^*,\quad K_i^*,\quad K_d^*
```

---

## 2. Optimizer Role

Optimizer는 controller를 직접 대체하는 것이 아니다.
Optimizer의 역할은 `PID Controller`가 사용할 gains를 찾는 것이다.

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

예를 들어 optimizer가 다음 candidate를 선택했다고 하자.

```text
Kp = 2.1
Ki = 0.8
Kd = 0.1
```

Simulation 결과로 $\omega(t)$, $V(t)$, $e(t)$를 얻고,
이를 이용해 $\mathcal{J}$를 계산한다.
Optimizer는 이 score를 기반으로 다음 candidate를 선택한다.

---

## 3. Random Search

`Random Search`는 gain search range 안에서 random candidate를 생성한다.

```text
Kp ∈ [Kp_min, Kp_max]
Ki ∈ [Ki_min, Ki_max]
Kd ∈ [Kd_min, Kd_max]
```

장점:

- 구현이 단순하다.
- optimizer baseline으로 사용하기 좋다.
- tuning range가 적절한지 빠르게 확인할 수 있다.

단점:

- 이미 평가한 결과를 다음 search에 적극적으로 활용하지 않는다.
- evaluation budget이 작으면 좋은 candidate를 놓칠 수 있다.

---

## 4. Genetic Algorithm

`Genetic Algorithm`은 여러 PID candidate를 population으로 관리한다.

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

이 프로젝트에서는 GA를 main method로 고정하지 않는다.
필요하면 Random Search와 Bayesian Optimization 사이의 comparison method로 추가한다.

---

## 5. Bayesian Optimization

`Bayesian Optimization`은 지금까지 평가한 관계를 이용한다.

```math
(K_p,K_i,K_d)\rightarrow\mathcal{J}
```

이 정보를 바탕으로 surrogate model을 만들고,

> 다음에는 어느 PID gains를 평가하는 것이 유리한가?

를 선택한다.

장점:

- Simulation cost가 큰 경우 evaluation budget을 아낄 수 있다.
- 이전 평가 결과를 이용해 더 똑똑하게 search한다.
- Random Search보다 적은 trial로 좋은 candidate를 찾을 가능성이 있다.

---

## 6. Main Experimental Story

v1의 중심 비교는 다음 순서로 진행한다.

```text
1. Manual PID baseline을 만든다.
2. Random Search로 자동 tuning baseline을 만든다.
3. Bayesian Optimization으로 sample-efficient tuning을 시도한다.
4. 세 controller의 response와 cost를 비교한다.
```

GA는 다음 조건에서 추가한다.

- Random Search와 Bayesian Optimization 사이의 비교가 필요할 때
- Population-based search를 설명하고 싶을 때
- 최종 report에서 optimizer diversity를 보여주고 싶을 때

---

## 7. What Optimizer Does Not Change

Optimizer는 다음을 바꾸지 않는다.

| Item | Reason |
|---|---|
| Motor parameters | 제어 대상인 plant의 물리적 특성 |
| Cost weights | v1 design choice |
| Reference signal | 모든 method를 같은 조건에서 비교해야 함 |
| Voltage limit | 실제 actuator constraint |

Optimizer가 바꾸는 것은 오직:

```math
K_p,\quad K_i,\quad K_d
```

이다.

자세한 실험 설정은 [Optimization Plan](../experiments/optimization_plan.md)에 정리한다.
