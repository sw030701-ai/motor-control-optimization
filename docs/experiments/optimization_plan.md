# Optimization Plan

이 문서는 PID optimization experiment를 실행하기 전에 정해야 할 조건을 정리한다.

목표는 모든 method를 같은 조건에서 비교하는 것이다.

---

## 1. Optimization Target

Optimizer가 찾는 값은 PID gains이다.

$$
\min_{K_p,K_i,K_d}\mathcal{J}(K_p,K_i,K_d)
$$

Cost function은 [Cost Function Design](../theory/cost_function.md)을 따른다.

---

## 2. Fixed Conditions

모든 optimizer는 동일한 simulation condition에서 비교한다.

| Item | Value |
|---|---|
| Motor parameter set | TBD |
| Reference speed $\omega_{\mathrm{ref}}$ | TBD |
| Simulation time $T$ | TBD |
| Sampling time $\Delta t$ | TBD |
| Voltage limit $V_{\max}$ | TBD |
| Initial speed $\omega(0)$ | TBD |
| Initial current $i(0)$ | TBD |
| Load torque $T_L$ | TBD |

다른 method를 비교할 때 위 조건이 바뀌면 optimizer 자체의 성능 차이를 해석하기 어렵다.

---

## 3. Motor Parameters

Nominal motor parameters는 open-loop validation 이후 확정한다.

| Parameter | Meaning | Value |
|---|---|---|
| $R$ | Armature resistance | TBD |
| $L$ | Armature inductance | TBD |
| $J_m$ | Rotor inertia | TBD |
| $b$ | Viscous friction | TBD |
| $K_t$ | Torque constant | TBD |
| $K_e$ | Back-EMF constant | TBD |

---

## 4. Gain Search Bounds

PID gain search range는 먼저 conservative하게 설정한다.

| Gain | Lower Bound | Upper Bound | Note |
|---|---:|---:|---|
| $K_p$ | TBD | TBD | Proportional gain |
| $K_i$ | TBD | TBD | Integral gain |
| $K_d$ | TBD | TBD | Derivative gain |

Search bound가 너무 좁으면 좋은 gains를 놓칠 수 있고,
너무 넓으면 unstable candidate가 많아져 evaluation budget이 낭비될 수 있다.

---

## 5. Evaluation Budget

각 method의 evaluation budget은 공정하게 설정한다.

| Method | Evaluation Budget | Note |
|---|---:|---|
| Manual PID | Fixed after tuning | Baseline |
| Random Search | TBD trials | Main optimizer baseline |
| Bayesian Optimization | TBD trials | Sample-efficient optimizer |
| Genetic Algorithm | TBD generations / population | Optional comparison |

Bayesian Optimization은 적은 trial에서 좋은 candidate를 찾는 것이 목적이므로,
Random Search와 같은 trial 수 또는 더 작은 trial 수에서 비교할 수 있다.

---

## 6. Candidate Evaluation Procedure

각 PID candidate는 같은 절차로 평가한다.

```text
1. Candidate gains 선택
2. Closed-loop motor simulation 실행
3. ω(t), e(t), V(t) 기록
4. Tracking cost 계산
5. Overshoot cost 계산
6. Control effort cost 계산
7. Total cost 𝓙 반환
```

Unstable candidate 또는 persistent saturation candidate는 높은 penalty를 줄 수 있다.

---

## 7. Results to Save

각 experiment에서 다음 정보를 저장한다.

| Item | Description |
|---|---|
| Best gains | $K_p^*$, $K_i^*$, $K_d^*$ |
| Best cost | $\mathcal{J}_{best}$ |
| Response curve | $\omega(t)$ |
| Control input | $V(t)$ |
| Tracking error | $e(t)$ |
| Trial history | candidate gains와 cost |

이 결과는 [Performance Comparison](performance_comparison.md)에 정리한다.
