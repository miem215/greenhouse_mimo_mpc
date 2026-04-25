# Greenhouse Climate Control: MIMO MPC with Disturbance Rejection

This repository contains a high-fidelity control system simulation for autonomous greenhouse climate management. It utilizes a **Model Predictive Controller (MPC)** paired with an **Augmented Kalman Filter** to regulate temperature and humidity while rejecting unmeasured external disturbances.

## Control Theory & Mathematical Framework

### 1. System Dynamics (MIMO State-Space)
The greenhouse is modeled as a discrete-time Linear Time-Invariant (LTI) system:

$$x(k+1) = Ax(k) + Bu(k) + d(k)$$
$$y(k) = Cx(k) + v(k)$$

Where:
* $x = [T, H]^T$ (Temperature, Humidity)
* $u = [P_{heat}, \theta_{vent}]^T$ (Heater Power, Vent Position)
* $d(k)$ represents unmeasured disturbances (Solar Gain).
* $v(k)$ is Gaussian measurement noise.

### 2. Augmented Kalman filter (Disturbance Estimation)
To achieve **offset-free tracking** in the presence of unmeasured force, in this case solar effect, the state vector was augmented with a disturbance bias $p$:

We extend the standard state-space model to include a third state, $p$, representing the persistent solar disturbance. We model the disturbance as a random walk ($p_{k+1} = p_k$), allowing the filter to track time-varying signals via the innovation loop.

$$\underbrace{\begin{bmatrix} T_{k+1} \\ H_{k+1} \\ p_{k+1} \end{bmatrix}}_{\hat{x}_{k+1}} = 
\begin{bmatrix} A & \begin{matrix} 1 \\ 0 \end{matrix} \\ 0 \ \ 0 & 1 \end{bmatrix} 
\begin{bmatrix} T_k \\ H_k \\ p_k \end{bmatrix} + 
\begin{bmatrix} B \\ 0 \end{bmatrix} u_k + w_k$$

This allows the Kalman Filter to estimate the "hidden" solar load, which is then used for **Feed-Forward Compensation** in the MPC. Figure below shows the modeled solar temprature disturbance and the estimated disturbance. 
<img width="640" height="480" alt="sun_estimate" src="https://github.com/user-attachments/assets/5da09e85-65f0-49ae-9ca7-da15238c6874" />

### 3. MPC Optimization Objective
The controller solves the following Quadratic Programming (QP) problem at each time step $k$ over a horizon $H$:

$$\min_{\Delta u} \sum_{i=1}^{H} \| \hat{x}_{k+i} - r_{k+i} \|_Q^2 + \| \Delta u_{k+i-1} \|_R^2$$

**Subject to:**
* System dynamics constraints.
* **Slew Rate Limits:** $\Delta u_{min} \leq \Delta u \leq \Delta u_{max}$ to ensure actuator longevity.

## Technical Performance

### Disturbance Rejection
By using the estimated bias $p$, we adjust the reference $r_{adj} = r - \hat{p}$. This ensures that even when solar radiation pushes the temperature up, the MPC "aims lower" to land exactly on the target setpoint.

## Project Structure

* **`plant.py`**: Physics engine with disturbance injection.
* **`kalman.py`**: 3-state Augmented Kalman Filter implementation.
* **`MPCoptimizer.py`**: Receding horizon controller using `qpsolvers`.
* **`simulator.py`**: Main execution loop and visualization.
---
