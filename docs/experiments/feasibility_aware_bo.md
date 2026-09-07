# Feasibility-Aware BO Methodology Study

이 문서는 `feature/feasibility-aware-bo` branch에서 추가한 optional methodology comparison을 정리한다.
현재 main story인 `Manual Baseline PID` vs `Constrained Classical PID` 비교는 그대로 유지한다.
이 branch의 목적은 기존 penalty-based constrained BO와 feasibility-aware BO를 같은 조건에서 비교하는 것이다.

---

## 1. Compared Methods

비교 대상은 아래 네 가지이다. Method 이름은 notebook/result artifact에서 그대로 쓰기 위해 영어 label을 유지한다.

| Method | Role | Feasibility Handling |
|---|---|---|
| Manual Baseline | Fixed benchmark | `02_pid_baseline_tuning.ipynb`에서 고정한 baseline gains를 같은 v1 constraints로 확인한다. |
| Constrained Random Search | Simple optimizer baseline | Uniform random candidates를 평가한 뒤 feasible candidates 중 raw total cost가 가장 작은 후보를 선택한다. |
| Penalty-based Constrained BO | Existing constrained BO baseline | Infeasible candidate에 `objective = 1e6`을 부여하고 single GP가 penalized objective를 학습한다. 다음 candidate는 EI로 선택한다. |
| Feasibility-aware BO | New optional study | Raw objective GP와 constraint GPs를 분리하고 `CEI = EI * P(feasible)`로 다음 candidate를 선택한다. |

---

## 2. Shared v1 Constraints

모든 optimizer는 같은 project-level v1 constraints를 사용한다.

| Constraint | Limit | Constraint Function |
|---|---:|---|
| Overshoot | `<= 10%` | `g_overshoot = overshoot - 0.10` |
| Steady-state error | `<= 2%` | `g_sse = steady_state_error - 0.02` |
| Settling time | `<= 2.0 s` | `g_settling = settling_time - 2.0` |
| Saturation fraction | `< 5%` | `g_saturation = saturation_fraction - 0.05` |
| Stability / validity | finite valid response | invalid candidate는 final selection 대상에서 제외 |

Constraint convention은 다음처럼 둔다.

```text
g_i(x) <= 0  -> feasible for that constraint
g_i(x) > 0   -> violation
```

`settling_time`이 `inf`로 나오면 constraint surrogate가 학습할 수 있도록 finite positive violation으로 clip한다.
Invalid 또는 non-finite simulation은 raw objective GP 학습에서 제외하고, constraint GP 쪽에는 finite positive violation으로 기록한다.

---

## 3. Penalty BO vs Feasibility-Aware BO

### Penalty-based Constrained BO

기존 constrained BO 구현은 하나의 scalar objective만 사용한다.

```text
if candidate is infeasible:
    objective = 1e6
else:
    objective = raw total cost J
```

즉 GP는 penalized objective를 직접 학습하고, Expected Improvement가 다음 candidate를 고른다.
다만 최종 선택은 penalty 값이 아니라 실제 simulation에서 `feasible=True`인 후보 중 raw total cost가 가장 작은 값으로 수행한다.

### Feasibility-Aware BO

새 branch 구현은 objective quality와 constraint satisfaction을 분리해서 모델링한다.

```text
Objective GP:
    (Kp, Ki, Kd) -> raw unpenalized J

Constraint GPs:
    (Kp, Ki, Kd) -> g_overshoot
    (Kp, Ki, Kd) -> g_sse
    (Kp, Ki, Kd) -> g_settling
    (Kp, Ki, Kd) -> g_saturation
```

각 candidate에 대해 constraint model은 다음 확률을 추정한다.

```math
P(g_i(x) \le 0)
```

그리고 constraint별 확률을 곱해 전체 feasibility probability를 계산한다.

```math
P_{\mathrm{feasible}}(x) = \prod_i P(g_i(x) \le 0)
```

Acquisition function은 다음과 같다.

```math
CEI(x) = EI(x) \times P_{\mathrm{feasible}}(x)
```

이 product는 constraint model 사이의 independence approximation을 사용한다.
최종 선택은 surrogate prediction이 아니라 실제로 simulate한 `feasible=True` candidates 중 raw total cost `J`가 가장 작은 후보로 한다.

---

## 4. Fixed Comparison Conditions

Comparison notebook은 기존 constrained optimization workflow와 같은 조건을 사용한다.

| Item | Value |
|---|---:|
| Reference speed | `12.60 rad/s` |
| Voltage limit | `12.0 V` |
| Simulation time | `10.0 s` |
| Sampling time | `0.001 s` |
| Load torque | `0.0` |
| Random seed | `42` |
| Trial budget per optimizer | `80` |
| BO initial random design | `8` |
| Acquisition pool size | `1200` |

Search bounds:

| Gain | Lower | Upper |
|---|---:|---:|
| `K_p` | `0.20` | `1.50` |
| `K_i` | `0.50` | `4.00` |
| `K_d` | `0.00` | `0.01` |

Random Search, penalty-based BO, feasibility-aware BO는 같은 seed와 bounds를 사용한다.
두 BO method는 같은 initial random design을 공유한다.

---

## 5. Results Artifacts

실험 notebook은 다음 파일이다.

- `experiments/04_feasibility_aware_bo_comparison.ipynb`

Notebook 실행 결과는 다음 artifact로 저장한다.

- `results/tables/feasibility_aware_bo_comparison.csv`
- `results/tables/feasibility_aware_bo_comparison.json`
- `results/tables/feasibility_aware_bo_random_search_history.csv`
- `results/tables/feasibility_aware_bo_penalty_bo_history.csv`
- `results/tables/feasibility_aware_bo_history.csv`
- `results/tables/feasibility_aware_bo_optimizer_summary.csv`
- `results/figures/feasibility_aware_bo_speed_response.png`
- `results/figures/feasibility_aware_bo_control_voltage.png`
- `results/figures/feasibility_aware_bo_convergence.png`
- `results/figures/feasibility_aware_bo_feasible_counts.png`

---

## 6. Limitations

- `P_feasible`는 constraint-model prediction들이 서로 독립이라고 근사한다.
- 각 constraint model은 deterministic simulation metric을 대상으로 하는 GP regression surrogate이다.
- `80` trial budget에서는 결론이 seed와 budget에 영향을 받을 수 있다.
- Feasibility-aware BO는 feasibility modeling을 더 명확하게 만들지만, penalty BO나 Random Search보다 항상 우월하다는 보장은 아니다.
