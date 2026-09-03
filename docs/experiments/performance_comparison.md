# Performance Comparison

이 문서는 `Conventional PID`, `Optimized PID`, `RL Controller`의 성능을 같은 조건에서 비교하기 위한 template이다.

---

## 1. Comparison Principle

모든 controller는 동일한 조건에서 평가한다.

```text
Same motor model
Same reference input
Same simulation time
Same voltage limit
Same initial condition
Same cost function
```

그래야 controller 자체의 차이를 비교할 수 있다.

---

## 2. Controllers

비교 대상은 다음과 같다.

| Controller | Description |
|---|---|
| Conventional PID | Manual tuning으로 만든 baseline PID |
| Optimized PID | Random Search 또는 Bayesian Optimization으로 찾은 PID gains |
| RL Controller | State를 보고 action을 결정하는 learning-based controller |

---

## 3. Main Metrics

| Metric | Meaning |
|---|---|
| Total Cost | Weighted cost function $\mathcal{J}$ |
| Tracking Error | Reference speed와 실제 speed의 차이 |
| Overshoot | Reference speed를 초과한 정도 |
| Settling Time | Reference 근처로 안정화되는 시간 |
| Steady-State Error | Simulation 후반에 남아 있는 error |
| Control Effort | Voltage 사용량 |

---

## 4. Comparison Table

| Controller | Total Cost | Tracking Error | Overshoot | Settling Time | Steady-State Error | Control Effort |
|---|---:|---:|---:|---:|---:|---:|
| Conventional PID | TBD | TBD | TBD | TBD | TBD | TBD |
| Optimized PID - Random Search | TBD | TBD | TBD | TBD | TBD | TBD |
| Optimized PID - Bayesian Optimization | TBD | TBD | TBD | TBD | TBD | TBD |
| RL Controller | TBD | TBD | TBD | TBD | TBD | TBD |

---

## 5. Plots

최종 report에서는 최소한 다음 plot을 비교한다.

| Plot | Description |
|---|---|
| Speed response | $\omega(t)$ vs $\omega_{\mathrm{ref}}$ |
| Tracking error | $e(t)$ |
| Control input | $V(t)$ |
| Cost history | optimizer trial에 따른 best cost 변화 |

---

## 6. Interpretation Questions

결과를 해석할 때는 다음 질문을 확인한다.

1. Optimized PID는 conventional PID보다 total cost가 낮은가?
2. Cost가 낮아진 이유는 tracking, overshoot, control effort 중 무엇 때문인가?
3. 더 빠른 response가 과도한 voltage 사용으로 얻어진 것은 아닌가?
4. Nominal condition에서 좋은 controller가 robustness test에서도 좋은가?
5. RL controller는 PID보다 좋아졌는가, 아니면 다른 trade-off를 보이는가?
