from plant import BasePlant
from MPCoptimizer import MPCoptimizer
import numpy as np
import matplotlib.pyplot as plt
from kalman import KalmanFilter

# helper functions
def get_target_trajectory(t, hz):
    """
    Step 0-50 is Day (21, 50), Step 51+ is Night (16, 60).
    """
    target_stack = []
    
    for i in range(hz):
        future_t = t + i
        
        if future_t < 50:
            # Day Target
            target_i = np.array([[21], [50]])
        else:
            # Night Target
            target_i = np.array([[16], [60]])
            
        target_stack.append(target_i)
    return np.vstack(target_stack)

def calculate_rmse(true_val, test_val):
    return np.sqrt(np.mean((true_val - test_val)**2))

#~~~~~~~~~~~~~~~~~~Initialize the system ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
A = np.array([[0.95, 0.01], 
              [0.0,  0.9]])
B = np.array([[0.5, -0.01], 
              [-0.02, 0.5]])

init_st = np.array([15,40])

Q = np.array([[200, 0], 
              [0,  400]])
R = np.array([[50, 0],[0,100]])

hz = 10
time_steps = 100
#target = np.array([[21], [50]]) 

x_est_trj = []
u_trj = []
ref_trj= []
y_measurement = []
x_true_trj = []


# process nopise, uncertainty in the model
Q_kf = np.array([[0.0025, 0], 
                 [0, 0.01]])

# measurement noise
R_kf = np.array([[0.25, 0], 
                 [0, 1]])

testsystem = BasePlant(init_st, A,B)
kf_estimator = KalmanFilter(A, B, C=np.eye(2), Q=Q_kf, R=R_kf, initial_state=init_st)
controller = MPCoptimizer(testsystem, R, Q, hz)
std_devs = np.array([0.5, 1.0])

#~~~~~~~~~~~~~~~~~~~~~ Main function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
for i in range(time_steps):

    target = get_target_trajectory(i, hz)
    x_true = testsystem.x # the trueth
    noise = np.random.normal(0, std_devs).reshape(-1,1)
    y_sensor = x_true.reshape(-1,1) + noise # The noisy measurement
    solar_impact = 0* np.sin(np.pi * i / time_steps)

    # Kalman, Predict based on last u then Update the estimation with noisy y
    kf_estimator.predict(testsystem.u_prev)
    x_est = kf_estimator.update(y_sensor)

    # feed the estimation to MPC 
    delta_u = controller.solve(x_est,target) 
    testsystem.update(delta_u, solar_impact)
    
    x_true_trj.append(x_true.flatten())
    x_est_trj.append(x_est)
    u_trj.append(testsystem.u_prev.flatten()) 
    ref_trj.append(target[0:2].flatten())
    y_measurement.append(y_sensor)

x_est_trajectory = np.array(x_est_trj)
x_true_trajectory = np.array(x_true_trj)
u_trajectory = np.array(u_trj)
ref_trajectory = np.array(ref_trj)
y_ns_measurement = np.array(y_measurement)

# calculate the RMS error
rmse_noisy_temp = calculate_rmse(x_true_trajectory[:, 0], y_ns_measurement[:, 0])
rmse_kalman_temp = calculate_rmse(x_true_trajectory[:, 0], x_est_trajectory[:, 0])

# ~~~~~~~~~~~~~~~~~~~~~plot the graph~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
time = np.arange(len(x_true_trj))

# FIGURE 1: KALMAN FILTER PERFORMANCE
plt.figure(1)

# Temperature Subplot
plt.subplot(2, 1, 1)
plt.scatter(time, y_ns_measurement[:, 0], color='cyan', s=5, alpha=0.3, label='Raw Sensor (Noisy)')
plt.plot(time, x_true_trajectory[:, 0], 'k-', linewidth=1.5, label='True State (Physics)')
plt.plot(time, x_est_trajectory[:, 0], 'r-', linewidth=2, label='Kalman Estimate')
plt.title("Kalman Filter: Temperature Estimation", fontweight='bold')
plt.ylabel("Temp (°C)")
plt.legend(loc='upper right')
plt.grid(True, alpha=0.2)

# Humidity Subplot
plt.subplot(2, 1, 2)
plt.scatter(time, y_ns_measurement[:, 1], color='magenta', s=5, alpha=0.3, label='Raw Sensor (Noisy)')
plt.plot(time, x_true_trajectory[:, 1], 'k-', linewidth=1.5, label='True State (Physics)')
plt.plot(time, x_est_trajectory[:, 1], 'b-', linewidth=2, label='Kalman Estimate')
plt.title("Kalman Filter: Humidity Estimation", fontweight='bold')
plt.ylabel("Humidity (%)")
plt.xlabel("Time Steps")
plt.legend(loc='upper right')
plt.grid(True, alpha=0.2)

plt.tight_layout()

# FIGURE 2: MPC PERFORMANCE
plt.figure(2)
fig, (ax_track, ax_u) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Tracking Subplot
ax_track.plot(time, ref_trajectory[:, 0], 'k--', alpha=0.7, label='Temp Target')
ax_track.plot(time, x_est_trajectory[:, 0], 'r-', linewidth=2, label='Temp (MPC Input)')
ax_track.plot(time, ref_trajectory[:, 1], 'k:', alpha=0.7, label='Hum Target')
ax_track.plot(time, x_est_trajectory[:, 1], 'b-', linewidth=2, label='Hum (MPC Input)')
ax_track.set_title("MPC: Reference Tracking", fontweight='bold')
ax_track.set_ylabel("Climate States")
ax_track.legend(loc='center left', bbox_to_anchor=(1, 0.5))
ax_track.grid(True, alpha=0.2)

# Actuator Subplot
ax_u.step(time, u_trajectory[:, 0], color='darkorange', where='post', label='Heater')
ax_u.step(time, u_trajectory[:, 1], color='teal', where='post', label='Vent')
ax_u.set_title("MPC: Control Inputs (Actuators)", fontweight='bold')
ax_u.set_ylabel("Power/Opening (%)")
ax_u.set_xlabel("Time Steps")
ax_u.legend(loc='center left', bbox_to_anchor=(1, 0.5))
ax_u.grid(True, alpha=0.2)

plt.tight_layout()
plt.show()