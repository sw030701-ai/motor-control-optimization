# Constrained PID Optimization Plan

이 문서는 현재 main story에서 사용할 PID optimization experiment 조건을 정리한다.
비교 대상은 `Manual Baseline PID`와 `Constrained Classical PID` 두 controller이다.
과거 unconstrained optimization 결과는 history로 남기되, 현재 main comparison에는 포함하지 않는다.

---

## 1. Optimization Target

Optimizer가 찾는 값은 PID gains이다.

```math
\min_{K_p,K_i,K_d}\mathcal{J}(K_p,K_i,K_d)
\quad
\text{subject to v1 constraints}
```

즉 candidate response가 먼저 v1 constraints를 만족해야 하고,
feasible candidates 중에서 total cost가 가장 작은 gain을 선택한다.

```text
Feasibility first
      ↓
Optimization second
```

Cost function은 [Cost Function Design](../theory/cost_function.md)을 따른다.

---

## 2. Fixed Conditions

모든 optimizer와 baseline은 동일한 simulation condition에서 비교한다.

| Item | Value |
|---|---:|
| Motor parameter set | Literature-based nominal set |
| Reference speed $\omega_{\mathrm{ref}}$ | `12.60 rad/s` |
| Simulation time $T$ | `10.0 s` |
| Sampling time $\Delta t$ | `0.001 s` |
| Voltage limit $V_{\max}$ | `12.0 V` |
| Initial speed $\omega(0)$ | `0` |
| Initial current $i(0)$ | `0` |
| Load torque $T_L$ | `0` |
| Random seed | `42` |

다른 method를 비교할 때 위 조건이 바뀌면 optimizer 자체의 성능 차이를 해석하기 어렵다.

---

## 3. Motor Parameters

Nominal motor parameters는 open-loop validation 이후 아래 값으로 확정했다.

| Parameter | Meaning | Value |
|---|---|---:|
| $R$ | Armature resistance | `0.18644` |
| $L$ | Armature inductance | `0.0063` |
| $J_m$ | Rotor inertia | `0.013767` |
| $b$ | Viscous friction | `0.049813` |
| $K_t$ | Torque constant | `0.020375` |
| $K_e$ | Back-EMF constant | `0.020375` |

Source: Yüksek et al., `Table 1. DC motor parameters`, DOI `10.1177/00202940261442256`.

---

## 4. v1 Constraints

아래 constraints는 baseline에도 constrained optimization에도 동일하게 적용한다.

| Constraint | v1 Limit | Reason |
|---|---:|---|
| Overshoot $M_p$ | `<= 10%` | 과도응답 overshoot를 과도하지 않게 제한하는 project-level target. Baseline tuning에 이미 사용한 기준이며 universal standard가 아니라 v1 engineering choice이다. |
| Steady-state error $SSE$ | `<= 2%` | reference tracking의 정상상태 정확도를 보장한다. Optimizer가 control effort만 줄이면서 reference 아래에 머무는 해를 선택하는 것을 막는다. |
| Settling time $t_s$ | `<= 2.0 s` | 현재 문헌 기반 motor parameter, `10 s` simulation, baseline result `1.322 s`와 일관된 response-speed requirement이다. 초기 toy model의 `0.5 s` 기준이 아니라 현재 repo 기준은 `2.0 s`이다. |
| Saturation fraction | `< 5%` | actuator voltage limit에 장시간 붙는 controller를 피한다. 순간적인 clipping과 persistent saturation을 구분하며 baseline tuning의 `saturation_fraction < 0.05`와 일치한다. |
| Stability / divergence | finite, non-divergent response | 물리적으로 유효하지 않은 candidate를 제거한다. |

---

## 5. Gain Search Bounds

PID gain search range는 baseline 주변을 포함하면서도 너무 넓지 않게 설정한다.

| Gain | Lower Bound | Upper Bound | Reason |
|---|---:|---:|---|
| $K_p$ | `0.20` | `1.50` | baseline `0.80`을 포함하고, 너무 작은 proportional response와 과도하게 aggressive한 response를 동시에 피한다. |
| $K_i$ | `0.50` | `4.00` | baseline `2.00`을 중심으로 SSE를 줄일 수 있는 범위를 열어두되, integral windup 위험을 제한한다. |
| $K_d$ | `0.00` | `0.01` | baseline `0.002`보다 조금 넓게 열어 damping 효과를 탐색하되, derivative gain이 과도하게 커지는 것을 피한다. |

이 bounds는 universal standard가 아니라 현재 v1 motor/reference 조건에 맞춘 conservative design choice이다.

---

## 6. Evaluation Budget and Final Selection

v1에서는 두 constrained classical method를 같은 evaluation budget으로 실행한다.

| Method | Evaluation Budget | Role |
|---|---:|---|
| Constrained Random Search | `80 trials` | Simple constrained search baseline |
| Constrained Bayesian Optimization | `80 trials` | Sample-efficient constrained optimizer |

최종 `Constrained Classical PID`는 두 method의 feasible best candidate 중
실제 total cost $\mathcal{J}$가 더 작은 controller로 선택한다.

Bayesian Optimization은 scalar objective가 필요하므로 infeasible candidate에는 큰 penalty를 준다.
단, 최종 선택은 penalty가 아니라 feasible candidates 중 실제 cost 기준으로 수행한다.

---

## 7. Candidate Evaluation Procedure

각 PID candidate는 같은 절차로 평가한다.

```text
1. Candidate gains 선택
2. Closed-loop motor simulation 실행
3. omega(t), e(t), V(t) 기록
4. Response metrics 계산
5. v1 constraints 확인
6. Feasible이면 total cost J 비교
7. Infeasible이면 최종 후보에서 hard reject
```

---

## 8. Results to Save

각 experiment에서 다음 정보를 저장한다.

| Item | Description |
|---|---|
| Best gains | $K_p^*$, $K_i^*$, $K_d^*$ |
| Best cost | $\mathcal{J}_{best}$ |
| Feasibility table | v1 constraints 만족 여부 |
| Response curve | $\omega(t)$ |
| Control input | $V(t)$ |
| Trial history | candidate gains, metrics, feasibility, objective |

이 결과는 [Performance Comparison](performance_comparison.md)에 정리한다.
