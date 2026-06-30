# Greenhouse Climate and Irrigation Management
This repository contains a control system simulation for autonomous greenhouse climate and irrigation management. It utilizes a Model Predictive Controller (MPC) paired with a Kalman Filter to regulate temperature, humidity, and multi-layer soil moisture.

The base system is modeled as a 4x2 Linear Time-Invariant (LTI) system. To realistically simulate physical phenomena—such as water traveling through irrigation pipes and percolating through soil layers—the state-space is mathematically augmented to 7 dimensions using shift registers.

## Project Structure

plant.py: Physics engine simulating the greenhouse environment and disturbances.

augment_delay.py: Matrix augmentation logic to convert physical delays into state-space shift registers.

extract_matrices.py: Matrix definitions and generation of the final 7x7 system.

kalman.py: 7-state Kalman Filter for observing physical and hidden delay states.

MPCoptimizer.py: Receding horizon optimal controller using Quadratic Programming (osqp).

simulator.py: Main execution loop and visualization.

## Control Theory & Mathematical Framework

1. System Dynamics (MIMO State-Space with Delays)

The base physical greenhouse is modeled with 4 states and 2 inputs:

States ($x_{phys}$): $[T, H, M_{surf}, M_{deep}]^T$ (Temperature, Humidity, Surface Moisture, Deep Moisture)

Inputs ($u$): $[P_{heat}, P_{pump}]^T$ (Heater Power, Water Pump)

Modeling Delays via State Augmentation:
In reality, actuators do not affect the environment instantly. To model a 2-step delay for the water pump and a 1-step percolation delay between the surface and deep soil, the state vector is expanded to include 3 hidden "memory" states ($x_{aug} \in \mathbb{R}^7$).

$$x(k+1) = A_{final}x(k) + B_{final}u(k) + d(k)$$

$$y(k) = C_{final}x(k) + v(k)$$

The hidden states mathematically act as a shift register, holding control actions and draining water in transition before applying them to the physical states.

2. Kalman Filter (State & Delay Estimation)

The Kalman Filter is designed to track all 7 states. Even though we only have sensors for the 4 physical states ($C_{final}$ is a $4 \times 7$ matrix), the filter mathematically reconstructs the unmeasured water currently "in transit" through the pipes and soil layers.

$$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K (y_k - C_{final} \hat{x}_{k|k-1})$$

Tuning Strategy: The process noise ($Q$) is tuned relatively low, reflecting the high physical "inertia" of the greenhouse (temperature and soil moisture change slowly). By trusting the $A_{final}$ physics matrix over the noisy sensor data ($R$), we achieve a highly stable state estimate, which is crucial for the MPC's long-term predictions.

3. MPC Optimization Objective

The controller solves a Quadratic Programming (QP) problem at each time step $k$ over a prediction horizon $H$. It minimizes the tracking error across all physical states while penalizing excessive actuator effort.

$$\min_{u} \sum_{i=1}^{H} \left( \| \hat{x}_{k+i} - r_{k+i} \|_Q^2 + \| u_{k+i-1} \|_R^2 \right)$$

Subject to physical actuator limits: $0 \leq u \leq 100$.

To account for steady-state offsets caused by system underactuation and external disturbances (like solar gain), the MPC includes a dynamic bias compensation mechanism that aggressively pushes the target trajectory against the persistent error.

## Simulation Results & System Analysis

A key outcome of this project is demonstrating that an optimal controller (MPC) is ultimately bound by the physical constraints (plant dynamics) of the system. Rather than forcing perfect but physically impossible tracking, the simulation mathematically exposes several real-world limitations of greenhouse environments:

1. Underactuation and State Coupling

The expanded greenhouse model is underactuated. We model 4 physical states but only actuate 2 inputs.

The Humidity Coupling: The system lacks a dedicated humidifier or vent. Steady-state row analysis of the $A$ matrix reveals that humidity is mathematically coupled to Temperature and Surface Moisture:


$$Hum_{ss} = 0.5 \cdot Temp_{ss} + 0.25 \cdot Surf_{ss}$$

Optimal Compromise: When the MPC is given conflicting targets (e.g., high humidity but low temperature), it performs an optimal mathematical compromise. The resulting steady-state offsets (Temperature settling at ~40°C while aiming for 22°C to support the humidity target) are not controller errors, but rather proof of the system's physical constraints.

2. Dominant Eigenvalues and Physical Inertia

The system exhibits heavily delayed responses in the deep soil layer due to percolation dynamics.

Eigenvalue Analysis: By analyzing the eigenvalues of the augmented $7 \times 7$ $A$ matrix, the system's dominant pole is identified at $\lambda = 0.95$.

First-Order Lag: Because water percolates downwards with no capillary feedback loop upward, the deep soil acts as an isolated first-order lag filter. The MPC correctly predicts this high inertia, proving that the deep soil cannot be rapidly saturated without severely flooding the surface layer.

3. Steady-State Moisture Constraints

The modeled percolation dynamics dictate a strict steady-state ratio between the surface and deep soil. By solving the discrete-time physics equation for the unforced deep soil layer:

$$Deep_{k+1} = 0.1 \cdot Surf_k + 0.95 \cdot Deep_k$$

We find that at a settled steady state, $Deep_{ss} = 2 \cdot Surf_{ss}$. The reference trajectories in the simulation were successfully updated to respect this natural moisture gradient (e.g., $Surf = 40, Deep = 80$), allowing the MPC to track them with zero steady-state soil error.

## Future Roadmap

This repository serves as a foundational framework for MIMO greenhouse control. Future expansions could take several directions:

🧠 Advanced Control Strategies

Integral Action: Formally augment the MPC state vector with the cumulative output tracking error to introduce true integral action, replacing the current simple bias compensation.

Soft Constraints & Prioritization: Modify the $Q$ weight matrix to implement strict target prioritization (e.g., heavily prioritizing Temperature tracking over Humidity to prevent crop freezing, while safely allowing the MPC to ignore the resulting coupled humidity offset).

Non-Linear MPC (NMPC): Transition the plant from LTI matrices to non-linear differential equations using solvers like CasADi (e.g., making evaporation rates a non-linear function of both temperature and humidity).

🛠️ Plant & Hardware Upgrades

Full Actuation Matrix: Expand the $B$ matrix to include the Heater, Pump, Vent, and an active Humidifier. This will fully decouple Temperature and Humidity, allowing the MPC to track all 4 references independently without steady-state compromise.

Drainage Dynamics: Implement a variable deep-soil drainage coefficient to model active sub-surface drainage systems.

📈 System Identification

Data-Driven Matrices: Replace the hardcoded theoretical $A$ and $B$ matrices with matrices derived from real-world IoT sensor data using System Identification techniques (e.g., ARMAX models or SINDy).