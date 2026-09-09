# RL Direct Voltage Control Experiment

이 실험은 PID gain tuning을 하지 않는다.
RL agent가 motor state를 직접 관찰하고 매 time step마다 voltage command를 출력한다.

```text
[e_t, omega_t, i_t] -> TD3 Agent -> V_t -> DC Motor
```

## Fixed Conditions

기존 main 실험과 같은 nominal DC motor, reference speed, voltage limit을 사용한다.

| Item | Value |
|---|---|
| Motor | `nominal_dc_motor_params()` |
| Reference | `12.6 rad/s` |
| Voltage limit | `-12 V <= V_t <= 12 V` |
| Evaluation metrics | `compute_cost()` and `is_feasible_v1()` |

## State

```math
s_t=
\begin{bmatrix}
e_t\\
\omega_t\\
i_t
\end{bmatrix}
```

| Element | Meaning |
|---|---|
| $e_t$ | $\omega_{ref}-\omega_t$ |
| $\omega_t$ | Current motor speed |
| $i_t$ | Armature current |

## Action

```math
a_t=V_t
```

The environment clips the action:

```math
V_t=\mathrm{clip}(a_t,-12,12)
```

## Reward

The reward mirrors the existing PID optimization cost philosophy:

```math
r_t=
-\left[
0.60\left(\frac{e_t}{\omega_{ref}}\right)^2
+0.25\left(\max\left(0,\frac{\omega_t-\omega_{ref}}{\omega_{ref}}\right)\right)^2
+0.15\left(\frac{V_t}{V_{max}}\right)^2
\right]
```

This keeps the final comparison aligned with the existing tracking,
overshoot, and control-effort metrics.

## Algorithm

The implementation uses `TD3`.
TD3 is selected because direct voltage control has a continuous action space,
and the action is only one-dimensional.
Compared with SAC, TD3 keeps the implementation smaller while still matching
the state-dependent continuous control problem.

## Outputs

Run:

```bash
python experiments/04_rl_direct_voltage_control.py --episodes 100
```

The script writes the final comparison table:

```text
results/tables/direct_voltage_rl_comparison.csv
```

The table compares:

| Controller | Meaning |
|---|---|
| Manual PID | Human-selected baseline PID gains |
| Optimized PID | Best constrained PID from main optimization |
| RL Direct Control | TD3 policy outputting voltage directly |

## Initial Result

The initial 100-episode TD3 run did not satisfy the v1 feasibility criteria.

| Controller | J_total | Steady-state error [%] | Saturation [%] | Feasible |
|---|---:|---:|---:|---|
| Manual PID | 0.03818 | 0.00000 | 0.000 | True |
| Optimized PID | 0.03670 | 0.00000 | 0.000 | True |
| RL Direct Control | 0.07485 | 45.84141 | 0.180 | False |

The learned RL policy remained below the target speed:

```text
omega_final = 6.824 rad/s
omega_ref   = 12.600 rad/s
```

This result suggests that, under this small training budget and simple state
definition, TD3 direct voltage control is less reliable than the constrained
PID approaches. The comparison is still useful because it shows a practical
limitation of applying RL directly: the method needs more training design,
reward shaping, and stability checks before it can replace optimized PID in
this DC motor setup.
