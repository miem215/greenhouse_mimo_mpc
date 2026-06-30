import numpy as np
import matplotlib.pyplot as plt
from plant import BasePlant
from kalman import KalmanFilter
from mpcoptimizer import MPCoptimizer

def get_meaningful_target(step):
    """
    Returns [T_ref, H_ref, S_ref, D_ref]
    """
    # Day phase: Step 0 to 100
    if 0 <= (step % 200) < 100:
        return np.array([[22.0], [65.0], [40], [80]]) 
    # Night phase: Step 100 to 200
    else:
        return np.array([[17.0], [80.0], [30], [60]])
# --- 1. CONFIGURATION ---
hz = 10
time_steps = 150
# Initial state (7 elements: [T, H, Surf, Deep, u_pump_delayed1, u_pump_delayed2, Layer1])
initial_state = np.zeros(7) 

# --- 2. INITIALIZATION (Assuming A_final, B_final, C_final are pre-loaded) ---
# Replace these with your actual extracted matrices
A_final = np.array([[ 0.9 ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ],
 [ 0.1 ,  0.8 ,  0.05,  0.  ,  0.  ,  0.  ,  0.  ],
 [-0.05, -0.02,  0.9 ,  0.  ,  0.  ,  0.6 ,  0.  ],
 [ 0.  ,  0.  ,  0.  ,  0.95,  0.  ,  0.  ,  1.  ],
 [ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ],
 [ 0.  ,  0.  ,  0.  ,  0.  ,  1.  ,  0.  ,  0.  ],
 [ 0.  ,  0.  ,  0.1 ,  0.  ,  0.  ,  0.  ,  0.  ]])
B_final = np.array([[0.5, 0. ],
 [0. , 0. ],
 [0. , 0. ],
 [0. , 0. ],
 [0. , 1. ],
 [0. , 0. ],
 [0. , 0. ]])
C_final = np.array([[1., 0., 0., 0., 0., 0., 0.],
 [0., 1., 0., 0., 0., 0., 0.],
 [0., 0., 1., 0., 0., 0., 0.],
 [0., 0., 0., 1., 0., 0., 0.]])


plant = BasePlant(initial_state, A_final, B_final)
kf = KalmanFilter(A_final, B_final, C_final, Q=np.eye(7)*0.01, R=np.eye(4)*0.1, initial_state=initial_state)

Q = np.diag([10000.0, 100.0, 10000.0, 1000.0])
R = np.eye(2) * 0.1
mpc = MPCoptimizer(A_final, B_final, C_final, R, Q, hz)

# --- 3. SIMULATION LOOP ---
x_est_trj, u_trj, ref_trj = [], [], []

for i in range(time_steps):
    # Get Targets (Adjusted for 4 states)
    current_target = get_meaningful_target(i)
    
    # If your MPC uses a horizon (hz), you need a stack of targets for i to i+hz
    target_stack = np.zeros((4 * hz, 1))
    for h in range(hz):
        target_stack[h*4 : (h+1)*4] = get_meaningful_target(i + h)
    
    # Measurement
    y_measured = (C_final @ plant.x) + np.random.normal(0, 0.05, (4, 1))
    
    # Filter
    kf.predict(plant.u_prev)
    kf.update(y_measured)
    
    # Optimize
    u_sequence = mpc.solve(kf.x_hat, target_stack)
    
    # Update Plant
    plant.update(u_sequence[0:2], solar_gain=0.1)
    
    # Store
    x_est_trj.append(kf.x_hat.flatten())
    u_trj.append(plant.u_prev.flatten())
    ref_trj.append(current_target.flatten())

    if i % 10 == 0:
        print(f"Step {i} | Surf Est: {kf.x_hat[2,0]:.2f} | Target: {current_target[2,0]:.2f} | Pump Cmd: {plant.u_prev[1,0]:.2f}")

# --- 4. PLOTTING ---
x_est_trj = np.array(x_est_trj)
u_trj = np.array(u_trj)
ref_trj = np.array(ref_trj)

plt.figure(figsize=(14, 10))

# State 0: Temperature
plt.subplot(2, 2, 1)
plt.plot(ref_trj[:, 0], 'k--', label='Ref (Temp)')
plt.plot(x_est_trj[:, 0], 'b-', label='Est (Temp)')
plt.title("Temperature")
plt.legend(); plt.grid(True)

# State 1: Humidity
plt.subplot(2, 2, 2)
plt.plot(ref_trj[:, 1], 'k--', label='Ref (Hum)')
plt.plot(x_est_trj[:, 1], 'c-', label='Est (Hum)')
plt.title("Humidity")
plt.legend(); plt.grid(True)

# State 2: Surface Moisture
plt.subplot(2, 2, 3)
plt.plot(ref_trj[:, 2], 'k--', label='Ref (Surf)')
plt.plot(x_est_trj[:, 2], 'g-', label='Est (Surf)')
plt.title("Surface Moisture")
plt.legend(); plt.grid(True)

# State 3: Deep Moisture
plt.subplot(2, 2, 4)
plt.plot(ref_trj[:, 3], 'k--', label='Ref (Deep)')
plt.plot(x_est_trj[:, 3], 'r-', label='Est (Deep)')
plt.title("Deep Moisture")
plt.legend(); plt.grid(True)

plt.tight_layout()
plt.show()