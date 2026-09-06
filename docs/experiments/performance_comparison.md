# Performance Comparison

이 문서는 `Manual Baseline PID`와 `Constrained Classical PID`의 성능을 같은 조건에서 비교하기 위한 template이다.
현재 main story에서는 unconstrained PID optimization을 제외한다.

---

## 1. Comparison Principle

모든 controller는 동일한 조건에서 평가한다.

```text
Same motor model
Same reference input
Same simulation time
Same voltage limit
Same initial condition
Same load torque
Same cost function
Same v1 constraints
```

그래야 controller 자체의 차이를 비교할 수 있다.

---

## 2. Controllers

비교 대상은 다음 두 controller이다.

| Controller | Description |
|---|---|
| Manual Baseline | `02_pid_baseline_tuning.ipynb`에서 sequential manual tuning으로 고정한 PID |
| Constrained Classical | Constrained Random Search와 Constrained Bayesian Optimization 중 feasible best J가 더 낮은 PID |

Constrained Random Search와 Constrained Bayesian Optimization은 최종 controller를 고르기 위한
내부 method 후보이다. 최종 비교표와 response plot에는 `Manual Baseline`과
선택된 `Constrained Classical`만 표시한다.

---

## 3. Main Metrics

| Metric | Meaning |
|---|---|
| `J_total` | Weighted cost function $\mathcal{J}$ |
| `J_tracking` | ITSE 기반 tracking cost |
| `J_overshoot` | Overshoot cost |
| `J_control` | Normalized voltage effort |
| `overshoot_percent` | Reference speed를 초과한 정도 |
| `settling_time` | Reference 근처로 안정화되는 시간 |
| `steady_state_error_percent` | Simulation 후반에 남아 있는 error |
| `voltage_max_abs` | 최대 사용 voltage |
| `saturation_percent` | Voltage limit 근처에 머문 sample 비율 |
| `feasible` | v1 constraints 만족 여부 |

---

## 4. v1 Constraint Satisfaction

최종 비교에 들어가는 controller는 모두 아래 조건을 만족해야 한다.

| Constraint | v1 Limit |
|---|---:|
| Overshoot $M_p$ | `<= 10%` |
| Steady-state error $SSE$ | `<= 2%` |
| Settling time $t_s$ | `<= 2.0 s` |
| Saturation fraction | `< 5%` |
| Stability / divergence | finite, non-divergent response |

Cost가 낮더라도 constraints를 만족하지 못하면 final comparison controller로 선택하지 않는다.

---

## 5. Comparison Table

| Controller | Kp | Ki | Kd | J_total | J_tracking | J_overshoot | J_control | Overshoot % | Settling Time | SSE % | Max Voltage | Saturation % | Feasible |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Manual Baseline | `0.8000` | `2.0000` | `0.0020` | `0.038177` | `0.000114` | `0.000000` | `0.254054` | `0.000` | `1.322 s` | `0.000000` | `10.243 V` | `0.000` | True |
| Constrained Classical | `0.2151` | `0.9965` | `0.0009` | `0.036702` | `0.000611` | `0.000000` | `0.242238` | `0.009` | `1.547 s` | `0.000000` | `6.002 V` | `0.000` | True |

The current constrained classical controller is selected by `03_pid_optimization.ipynb`
from Constrained Bayesian Optimization.

---

## 6. Plots

최종 report에서는 다음 plot을 비교한다.

| Plot | Description |
|---|---|
| Speed response | `Manual Baseline` vs `Constrained Classical`, with $\omega_{\mathrm{ref}}$ line |
| Control input | `Manual Baseline` vs `Constrained Classical`, with $\pm V_{\max}$ lines |
| Constraint satisfaction | 두 controller가 v1 constraints를 모두 만족하는지 확인 |

Optimizer cost history는 method-selection evidence로 저장할 수 있지만,
main performance comparison plot에는 두 최종 controller만 표시한다.

---

## 7. Interpretation Questions

결과를 해석할 때는 다음 질문을 확인한다.

1. Constrained Classical PID는 v1 constraints를 모두 만족하는가?
2. Manual Baseline보다 total cost가 낮은가?
3. Cost가 낮아진 이유는 tracking, overshoot, control effort 중 무엇 때문인가?
4. 더 낮은 control effort가 느린 settling time과 trade-off를 만드는가?
5. Nominal condition에서 좋은 controller가 robustness test에서도 좋은가?
