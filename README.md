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

## 🛡️ State Estimation (Kalman Filter)

In a real-world greenhouse, sensors are rarely perfect. Environmental interference, electrical noise, and placement variance can cause "jitter" in raw data. A control system that reacts to every small sensor spike would suffer from **actuator chattering**—turning heaters or pumps on and off rapidly—which leads to mechanical fatigue and inefficient energy consumption.

I implemented a **Kalman Filter** to bridge the gap between noisy measurements and the MPC optimizer, ensuring the controller operates on a statistically optimal estimate of the true state.

### 1. The Stochastic Logic
The filter maintains a running "belief" of the greenhouse state by balancing two competing sources of information:
1.  **The Physics Model:** A mathematical prediction of how temperature and humidity should behave based on the previous state and known control inputs.
2.  **The Sensor Data:** Real-time observations that provide the ground truth but contain stochastic noise.


### 2. The Recursive Process
The implementation follows the classic two-step recursive loop:

#### **Step 1: Prediction (A-Priori)**
The filter uses the state-space matrices ($A, B$) to project the current state and error covariance ($P$) forward in time.
$$\hat{x}_{k|k-1} = A\hat{x}_{k-1|k-1} + Bu_{k-1}$$
$$P_{k|k-1} = AP_{k-1|k-1}A^T + Q_{kf}$$

#### **Step 2: Update (A-Posteriori)**
When a new measurement ($y_k$) arrives, the filter calculates the **Kalman Gain ($K$)**. This gain acts as a weighting factor: if the sensors are noisy, the filter trusts the model more; if the model is uncertain, it trusts the sensors.
$$K_k = P_{k|k-1}C^T(CP_{k|k-1}C^T + R_{kf})^{-1}$$
$$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k(y_k - C\hat{x}_{k|k-1})$$


### 3. Tuning & Robustness
* **Process Noise ($Q_{kf}$):** Configured to account for model inaccuracies, such as unexpected drafts or unmodeled heat exchange.
* **Measurement Noise ($R_{kf}$):** Set based on the variance of the physical sensors used in the simulation.

By integrating this estimator, the MPC receives a **smooth state trajectory**, resulting in stable control actions and robust performance even in the presence of significant measurement uncertainty.

---
