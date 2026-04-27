# Greenhouse Climate Control: MIMO MPC with Disturbance Rejection

This repository contains a control system simulation for autonomous greenhouse climate management. It utilizes a **Model Predictive Controller (MPC)** paired with an **Augmented Kalman Filter** to regulate temperature and humidity while rejecting unmeasured external disturbances. The system is modeled as a 2x2 LTI system, in reality the greenhouse dynamic system is far mode complecated. The purpose of this project is not to showcase a precise modeling of greenhouse, rather it is to demonstrate the implmentation Observer-Based MPC and Disturbance Rejection.
## Project Structure

* **`plant.py`**: Physics engine with disturbance injection.
* **`kalman.py`**: 3-state Augmented Kalman Filter implementation.
* **`MPCoptimizer.py`**: Receding horizon controller using `qpsolvers`.
* **`simulator.py`**: Main execution loop and visualization.

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
To achieve **offset-free tracking** in the presence of unmeasured force, in this case solar effect, the state vector was augmented with a disturbance bias $p$. We model the disturbance as a random walk ($p_{k+1} = p_k$). This augmented state-space model allows the Kalman Filter to estimate the "hidden" solar load, which is then used for Feed-Forward Compensation in the MPC.

$$
\underbrace{
\begin{bmatrix} 
T_{k+1} \\ 
H_{k+1} \\ 
p_{k+1} 
\end{bmatrix}}_{\hat{x}_{k+1}} = 
\begin{bmatrix} 
A & \begin{matrix} 1 \\ 0 \end{matrix} \\ 
0 \ \ 0 & 1 
\end{bmatrix} 
\begin{bmatrix} 
T_k \\ 
H_k \\ 
p_k 
\end{bmatrix} + 
\begin{bmatrix} 
B \\ 
0 
\end{bmatrix} u_k + w_k
$$

Figures below shows the Kalman filter performance for estimating the states and the estimated disturbance respectively.
<img width="924" height="701" alt="fig1_kalman" src="https://github.com/user-attachments/assets/7ef05219-b4f5-435c-8b9d-057c89dee3b1" />

<img width="640" height="480" alt="sun_estimate" src="https://github.com/user-attachments/assets/5da09e85-65f0-49ae-9ca7-da15238c6874" />

The uncertainty matrices were chosen as follows. Overal the process noise are much less than the sensor noise, because there are 'iniertia' for the temperature and humidity, these states will not deviete a lot from the physics model. The disturbance gain $$Q_{22}$$ is relatively large, telling kalman filter that the disturbance can move quite fast. In my current simulation, the solar disturbance is modeled as a smooth sine wave. Mathematically, I could achieve a much smoother estimate by reducing $Q_{22}$ to match that slow frequency. However, I deliberately chose a larger $Q_{22}$ to ensure the filter remains responsive to edge cases

process noise 
Q_kf = np.array([[0.0025, 0, 0], 
                 [0, 0.01, 0],
                 [0, 0, 0.01]])

measurement noise
R_kf = np.array([[1.5, 0], 
                 [0, 1]])
                 

### 3. MPC Optimization Objective
By augmenting the Kalman Filter to estimate the solar disturbance ($\hat{p}_k$), we can determine the steady-state impact of the environmental load using the system's DC gain. Adjusting the reference trajectory in this manner ensures offset-free tracking, effectively canceling the disturbance before it can deviate the climate from the biological setpoint. The adjuested trajectory is expressed as:

$$T_{target, adj} = T_{target} - \frac{\hat{p}_k}{1 - A_{1,1}}$$

The controller solves the following Quadratic Programming (QP) problem at each time step $k$ over a horizon $H$

$$\min_{\Delta u} \sum_{i=1}^{H} \| \hat{x}_{k+i} - T_{target, adj}^{k+i} \|_Q^2 + \| \Delta u_{k+i-1} \|_R^2 $$

We solve for $$\Delta u$$, change of the control input, subject to **Slew Rate Limits:** $\Delta u_{min} \leq \Delta u \leq \Delta u_{max}$ to ensure smooth control actions.

### 4. Performance

The controller demonstrated effective setpoint tracking and smooth day-to-night transition (21°C/50% → 16°C/60%). Humidity converged to target with minimal overshoot. A residual steady-state temperature offset of approximately 2°C was observed, attributable to the absence of integral action in the MPC formulation (discussed in Future Work).


<img width="1484" height="1181" alt="fig2_mpc" src="https://github.com/user-attachments/assets/03aff46c-8aa0-4334-bafd-96529a2add23" />



**Future Work**: 

Integral action (eliminate steady-state error). The most impactful improvement. Augmenting the MPC state vector with the cumulative output tracking error introduces effective integral action, eliminating the steady-state offset observed in temperature tracking. This is standard practice in industrial MPC implementations.

---
