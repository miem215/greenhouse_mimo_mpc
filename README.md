# MIMO Greenhouse Climate Control: Augmented State MPC

## Project Overview
This repository features a **MIMO** Model Predictive Controller (MPC) designed to automate greenhouse climate conditions. The system regulates **Temperature** and **Humidity** by managing two coupled actuators: a heater and a humidifier.

The controller implements an **Incremental Control (Delta-Input) formulation**. By optimizing the *change* in control effort rather than absolute values.

---

## Key Features
* **MIMO State-Space Modeling**: Accurately handles the physical cross-coupling where heating affects humidity.
* **Augmented State Formulation**: Tracks control history as an internal state, enabling $\Delta u$ optimization.
* **Constrained Optimization**: Utilizes the **OSQP solver** via `qpsolvers` to respect physical limits (0-100% power) and slew-rate constraints.
* **Reference Trajectory Preview**: A 10-step look-ahead horizon allows the controller to anticipate scheduled changes (e.g., transitioning from Day to Night cycles).

---

## Mathematical Approach
The controller uses a discrete-time state-space model augmented to include the previous control input $u_{k-1}$ as a state:

$$X_{k+1} = \begin{bmatrix} A & B \\ 0 & I \end{bmatrix} X_k + \begin{bmatrix} B \\ I \end{bmatrix} \Delta u_k$$

By penalizing $\Delta u$ in the cost function ($R$ matrix), we prevent aggressive "chattering" of the equipment.


---

## Performance Visualization
The simulation demonstrates the controller's predictive nature. Notice how the actuators begin to adjust *prior* to the setpoint change at step 50, minimizing the error transition.

<img width="1000" height="800" alt="MPC" src="https://github.com/user-attachments/assets/5e4b4436-9cbe-4342-85de-4568032efe6c" />


---
