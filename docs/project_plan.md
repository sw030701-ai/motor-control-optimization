
# Motor Control Optimization - Project Plan

## Version

Project Plan v1

---

# 1. Project Goal

The goal of this project is to study and compare control methods for a DC motor speed control system.

The project will proceed through the following pipeline:

Motor Modeling  
→ PID Baseline  
→ Cost Function Design  
→ Classical Optimization  
→ Reinforcement Learning Control  
→ Performance Comparison  
→ Robustness Test  
→ Hardware / Robot Extension

The main objective is to understand how control parameters affect motor behavior and to automatically search for better control performance using optimization algorithms.

The first target system is a DC motor speed control system.

Future extensions may include reinforcement learning, physical motor hardware, and mobile robot trajectory control.

---

# 2. Main Research Questions

This project will focus on the following questions.

### Q1. How can a DC motor be represented mathematically?

The electrical and mechanical dynamics of the motor will be modeled using differential equations and state-space representation.

### Q2. How do PID gains affect motor speed control performance?

The proportional, integral, and derivative gains

Kp, Ki, Kd

will be used as controller parameters.

### Q3. Can optimization algorithms find better PID gains than manual tuning?

Several optimization methods will search for PID gains that minimize a predefined cost function.

### Q4. How does an optimized PID controller compare with a reinforcement learning controller?

Both methods will be evaluated using the same motor model and similar performance metrics.

### Q5. Do the controllers remain effective when the motor parameters or external load change?

Robustness tests will be performed using parameter variations and load disturbances.

---

# 3. System Definition

## 3.1 Target System

The initial target is an armature-controlled DC motor.

### Input

The control input is the motor voltage:

V(t)

### States

The motor states are defined as:

x1(t) = i(t)

x2(t) = omega(t)

Therefore,

x(t) = [i(t), omega(t)]^T

where:

- i(t): armature current
- omega(t): angular velocity

### Output

The controlled output is motor angular velocity:

y(t) = omega(t)

### Disturbance

The external load torque is represented by:

TL(t)

For the initial nominal simulation:

TL(t) = 0

The load torque will later be introduced during robustness testing.

---

# 4. DC Motor Modeling

The DC motor is modeled using an electrical equation and a mechanical equation.

## 4.1 Electrical Dynamics

Applying Kirchhoff's Voltage Law to the armature circuit gives:

V(t) = L di(t)/dt + R i(t) + Ke omega(t)

Therefore,

di(t)/dt
=
-(R/L)i(t)
-(Ke/L)omega(t)
+(1/L)V(t)

where:

- R: armature resistance
- L: armature inductance
- Ke: back-EMF constant

---

## 4.2 Mechanical Dynamics

The motor torque is proportional to current:

Tm(t) = Kt i(t)

The rotational dynamics are:

J domega(t)/dt
=
Kt i(t)
-b omega(t)
-TL(t)

Therefore,

domega(t)/dt
=
(Kt/J)i(t)
-(b/J)omega(t)
-(1/J)TL(t)

where:

- J: rotor inertia
- b: viscous friction coefficient
- Kt: motor torque constant
- TL: external load torque

---

# 5. State-Space Model

The state vector is:

x(t)
=
[i(t), omega(t)]^T

The system is represented as:

dx/dt = A x + B V + E TL

with

A =

[ -R/L      -Ke/L ]

[  Kt/J      -b/J ]

B =

[ 1/L ]

[  0  ]

E =

[  0   ]

[ -1/J ]

The output equation is:

y = Cx

where

C = [0  1]

Therefore,

y(t) = omega(t)

This state-space model will be implemented in Python and used as the main simulation environment.

---

# 6. PID Baseline Controller

The first controller will be a PID controller.

The tracking error is defined as:

e(t)
=
omega_ref(t)
-
omega(t)

The PID control law is:

u(t)
=
Kp e(t)
+
Ki integral(e(t)) dt
+
Kd de(t)/dt

The controller output u(t) corresponds to the motor input voltage V(t).

The PID gains are:

- Kp: proportional gain
- Ki: integral gain
- Kd: derivative gain

These three parameters determine the behavior of the controller.

The initial PID controller will be manually tuned and used as the baseline controller.

---

# 7. Optimization Variables

The optimization problem searches for the PID gain vector:

theta
=
[Kp, Ki, Kd]

The objective is to find:

theta*
=
[Kp*, Ki*, Kd*]

that produces the best motor control performance.

The optimization problem is defined as:

minimize J(Kp, Ki, Kd)

subject to:

Kp >= 0

Ki >= 0

Kd >= 0

The upper search bounds will be determined after the nominal DC motor parameters are selected and the initial simulations are performed.

---

# 8. Cost Function Design

A controller cannot be optimized unless "good control performance" is represented numerically.

The project therefore defines a cost function J.

A smaller value of J represents better control performance.

The v1 cost function considers three objectives:

1. Tracking performance
2. Overshoot
3. Control effort

The total cost is:

J
=
we J_tracking
+
wo J_overshoot
+
wu J_control

Initial weights:

we = 0.60

wo = 0.25

wu = 0.15

These values are initial design choices and may be adjusted after simulation experiments.

---

# 9. Tracking Cost

The speed tracking error is:

e(t)
=
omega_ref(t)
-
omega(t)

To compare errors independently of the reference speed, a normalized error is defined:

e_normalized(t)
=
[omega_ref(t) - omega(t)] / omega_ref(t)

The tracking cost uses the ITSE concept:

J_tracking
=
(1/T^2)
integral from 0 to T of
t * e_normalized(t)^2 dt

The time weighting penalizes errors that remain for a long time.

Therefore, a controller that reaches the target speed slowly receives a larger cost.

---

# 10. Overshoot Cost

The normalized overshoot is defined as:

Mp
=
(omega_max - omega_ref) / omega_ref

when

omega_max > omega_ref

The overshoot cost is:

J_overshoot = Mp^2

If there is no overshoot:

J_overshoot = 0

This term discourages controllers that produce excessive speed peaks.

---

# 11. Control Effort Cost

A controller should not achieve good tracking simply by continuously applying excessive voltage.

The normalized control input is:

u_normalized(t)
=
V(t) / Vmax

The control effort cost is:

J_control
=
(1/T)
integral from 0 to T of
u_normalized(t)^2 dt

This term penalizes unnecessarily large control inputs.

---

# 12. Final Cost Function v1

The first cost function used in the project is:

J
=
0.60 J_tracking
+
0.25 J_overshoot
+
0.15 J_control

The optimization goal is:

minimize J(Kp, Ki, Kd)

The optimizer receives candidate PID gains:

(Kp, Ki, Kd)

The motor simulation is executed using those gains.

The simulation produces:

omega(t)

V(t)

e(t)

These signals are used to calculate J.

The optimizer then proposes a new set of PID gains.

This process is repeated until a sufficiently good solution is found.

---

# 13. Optimization Loop

The main optimization loop is:

Optimizer  
→ PID gains  
→ PID Controller  
→ DC Motor Simulation  
→ Motor Response  
→ Cost Function  
→ Cost J  
→ Optimizer

In mathematical form:

(Kp, Ki, Kd)
→ simulation
→ J(Kp, Ki, Kd)

The final result is:

(Kp*, Ki*, Kd*)

which represents the best PID gains found by the selected optimization algorithm.

---

# 14. Classical Optimization Methods

The following methods will initially be considered.

## 14.1 Manual PID Tuning

Used as the baseline.

## 14.2 Random Search

Random PID gain combinations are evaluated.

This method provides a simple optimization baseline.

## 14.3 Genetic Algorithm

Candidate PID gain sets evolve using:

- selection
- crossover
- mutation

## 14.4 Bayesian Optimization

A surrogate model estimates the relationship:

(Kp, Ki, Kd) → J

and intelligently selects the next parameter combination to evaluate.

The optimization methods will be compared using:

- final cost
- number of simulations
- optimization time
- controller performance

---

# 15. Reinforcement Learning Control

Reinforcement learning will be introduced after the PID optimization stage is completed.

Unlike PID optimization, reinforcement learning does not necessarily search for fixed PID gains.

Instead, an RL agent observes the current motor state and selects a control action.

A possible RL state is:

st
=
[e(t), omega(t), i(t)]

A possible action is:

at = V(t)

or a normalized PWM duty command.

The RL interaction is:

state
→ RL agent
→ action
→ motor
→ next state

The reward function will follow a similar philosophy to the PID cost function.

For example:

rt
=
-alpha * e_normalized(t)^2
-beta * u_normalized(t)^2

The RL objective is to maximize cumulative reward.

PID optimization:

minimize J

RL control:

maximize cumulative reward

---

# 16. Performance Comparison

The following controllers will be compared:

1. Manually tuned PID
2. Optimized PID
3. Reinforcement learning controller

Evaluation metrics will include:

- tracking error
- overshoot
- settling time
- steady-state error
- control effort
- total cost
- robustness

---

# 17. Robustness Test

A controller that performs well only under nominal conditions may not work well in a real motor.

Therefore, robustness tests will be performed.

Possible tests include:

### Resistance variation

R → 1.2R

### Inertia variation

J → 1.3J

### External load torque

TL > 0

### Sensor noise

Noise may be added to measured motor speed.

The performance of the manual PID, optimized PID, and RL controller will be compared under these conditions.

---

# 18. Hardware Extension

After the simulation stage, the project may be extended to physical hardware.

Possible hardware components include:

- DC motor
- motor driver
- encoder
- current sensor
- Arduino or STM32
- power supply

The optimized controller will then be tested on a real motor.

---

# 19. Mobile Robot Extension

The motor control system may later be extended to a two-wheel mobile robot.

The robot will contain:

- left motor
- right motor
- wheel encoders
- motor drivers
- embedded controller

The control problem may then be extended from motor speed control to trajectory tracking.

Possible future optimization targets include:

- left/right motor gains
- path tracking controller gains
- energy usage
- trajectory tracking error

---

# 20. Optional Quantum Optimization Extension

Quantum optimization is not part of the core project.

It will be treated as an optional experimental extension.

Possible topics include:

- discretization of PID gains
- binary encoding
- QUBO formulation
- QAOA
- warm-start QAOA
- comparison with classical optimization

The main project must remain complete even without quantum hardware.

---

# 21. Project Pipeline

The current project pipeline is:

Motor Modeling  
→ PID Baseline  
→ Cost Function Design  
→ Classical Optimization  
→ RL Control  
→ Performance Comparison  
→ Robustness Test  
→ Hardware / Robot Extension

Optional extension:

QUBO / QAOA

---

# 22. Project v1 Decisions

The following decisions are fixed for Project Plan v1.

### Motor

Armature-controlled DC motor

### Control target

Motor angular velocity

### Input

Motor voltage V(t)

### States

Current i(t)

Angular velocity omega(t)

### Baseline controller

PID controller

### Optimization variables

Kp, Ki, Kd

### Optimization objective

Minimize:

J
=
0.60 J_tracking
+
0.25 J_overshoot
+
0.15 J_control

### Tracking metric

Normalized ITSE

### Disturbance

Load torque TL

### Main optimization methods

- Random Search
- Genetic Algorithm
- Bayesian Optimization

### Learning-based extension

Reinforcement Learning

### Optional experimental extension

QUBO / QAOA

---

# 23. Parameters To Be Determined

The following values are not yet fixed and will be determined in the next stage:

- R: armature resistance
- L: armature inductance
- J: rotor inertia
- b: viscous friction coefficient
- Kt: torque constant
- Ke: back-EMF constant
- Vmax: maximum motor voltage
- omega_ref: reference speed
- simulation time T
- PID gain search bounds

These parameters must be selected before the first DC motor simulation is implemented.

