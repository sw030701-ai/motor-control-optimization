# Feasibility-Aware BO Methodology Study

이 문서는 `feature/feasibility-aware-bo` branch에서 추가한 optional methodology comparison을 정리한다.
현재 main story인 `Manual Baseline PID` vs `Constrained Classical PID` 비교는 그대로 유지한다.
이 branch의 목적은 기존 penalty-based constrained BO와 feasibility-aware BO를 같은 조건에서 비교하는 것이다.

---

## 1. Compared Methods

| Method | Role | Feasibility Handling |
|---|---|---|
| Manual Baseline | Fixed benchmark | `02_pid_baseline_tuning.ipynb`에서 고정한 baseline gains를 같은 v1 constraints로 확인 |
| Constrained Random Search | Simple optimizer baseline | Uniform random candidates를 평가한 뒤 feasible candidates 중 raw total cost 최소값 선택 |
| Penalty-based Constrained BO | Existing constrained BO baseline | Infeasible candidate에 `objective = 1e6`을 부여하고 single GP가 penalized objective를 학습, EI로 다음 candidate 선택 |
| Feasibility-aware BO | New optional study | Raw objective GP와 constraint GPs를 분리하고 `CEI = EI * P(feasible)`로 다음 candidate 선택 |

---

## 2. Shared v1 Constraints

모든 optimizer는 같은 project-level v1 constraints를 사용한다.

| Constraint | Limit | Constraint Function |
|---|---:|---|
| Overshoot | `<= 10%` | `g_overshoot = overshoot - 0.10` |
| Steady-state error | `<= 2%` | `g_sse = steady_state_error - 0.02` |
| Settling time | `<= 2.0 s` | `g_settling = settling_time - 2.0` |
| Saturation fraction | `< 5%` | `g_saturation = saturation_fraction - 0.05` |
| Stability / validity | finite valid response | invalid candidates are not eligible for final selection |

Constraint convention:

```text
g_i(x) <= 0  -> feasible for that constraint
g_i(x) > 0   -> violation
```

If `settling_time` is infinite, the feasibility-aware implementation clips it to a finite positive violation for the constraint surrogate. Invalid or non-finite simulations are excluded from the raw objective GP and mapped to finite positive constraint violations.

---

## 3. Penalty BO vs Feasibility-Aware BO

### Penalty-based Constrained BO

The existing constrained BO implementation keeps a single scalar objective:

```text
if candidate is infeasible:
    objective = 1e6
else:
    objective = raw total cost J
```

The GP learns this penalized objective directly, and Expected Improvement chooses the next candidate. Final selection is still made from actually feasible candidates using raw total cost.

### Feasibility-Aware BO

The new branch implementation separates objective quality from constraint satisfaction:

```text
Objective GP:
    (Kp, Ki, Kd) -> raw unpenalized J

Constraint GPs:
    (Kp, Ki, Kd) -> g_overshoot
    (Kp, Ki, Kd) -> g_sse
    (Kp, Ki, Kd) -> g_settling
    (Kp, Ki, Kd) -> g_saturation
```

For each candidate, the constraint models estimate:

```math
P(g_i(x) \le 0)
```

Then:

```math
P_{\mathrm{feasible}}(x) = \prod_i P(g_i(x) \le 0)
```

and the acquisition function is:

```math
CEI(x) = EI(x) \times P_{\mathrm{feasible}}(x)
```

The product uses the standard independence approximation across constraint models.
Final selection is made only from actually simulated `feasible=True` candidates using raw total cost `J`.

---

## 4. Fixed Comparison Conditions

The comparison notebook uses the same conditions as the existing constrained optimization workflow.

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

Random Search, penalty-based BO, and feasibility-aware BO use the same seed and bounds. The two BO methods share the same initial random design.

---

## 5. Results Artifacts

The experiment is implemented in:

- `experiments/04_feasibility_aware_bo_comparison.ipynb`

The notebook saves:

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

- `P_feasible` assumes independent constraint-model predictions.
- Each constraint model is a GP regression surrogate over deterministic simulation metrics.
- With an `80`-trial budget, conclusions are seed- and budget-dependent.
- Feasibility-aware BO improves modeling clarity, but it does not guarantee universal dominance over penalty BO or Random Search.
