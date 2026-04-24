from plant import BasePlant
from MPCoptimizer import MPCoptimizer
import numpy as np
import matplotlib.pyplot as plt
from Kalman import KalmanFilter

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

#~~~~~~~~~~~~~~~~~~Initialize the system ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
A = np.array([[0.95, 0.01], 
              [0.0,  0.9]])
B = np.array([[0.5, -0.01], 
              [-0.02, 0.5]])

init_st = np.array([15,40])

Q = np.array([[200, 0], 
              [0,  500]])
R = np.array([[0.1, 0],[0,0.1]])

hz = 10
time_steps = 100
#target = np.array([[21], [50]]) 

x_trj = []
u_trj = []
ref_trj= []

R_kf = np.array([[0.25, 0], 
                 [0, 1.0]])

# We trust our model quite a bit
Q_kf = np.array([[0.01, 0], 
                 [0, 0.01]])

testsystem = BasePlant(init_st, A,B)
kf_estimator = KalmanFilter(A, B, C=np.eye(2), Q=Q_kf, R=R_kf, initial_state=init_st)
controller = MPCoptimizer(testsystem, R, Q, hz)

#~~~~~~~~~~~~~~~~~~~~~ Main function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
for i in range(time_steps):
    x_true = testsystem.x # the trueth
    y_sensor = x_true + np.random.normal(0, 0.5, size=x_true.shape) # The noisy measurement

    # Predict based on last u, then Update with noisy y
    kf_estimator.predict(testsystem.u_prev)
    x_est = kf_estimator.update(y_sensor)

    target = get_target_trajectory(i, hz)
    delta_u = controller.solve(x_est,target) 
    testsystem.update(delta_u)
    
    x_trj.append(testsystem.x)
    u_trj.append(testsystem.u_prev.flatten()) 
    ref_trj.append(target[0:2].flatten())

x_trajactory = np.array(x_trj)
u_trajectory = np.array(u_trj)
ref_trajectory = np.array(ref_trj)

# ~~~~~~~~~~~~~~~~~~~~~plot the graph~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
time = np.arange(len(x_trj))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Plot 1: States (Temperature and Humidity)
ax1.plot(time, x_trajactory[:, 0], 'r-', label='Temperature (°C)', linewidth=2)
ax1.plot(time, x_trajactory[:, 1], 'b-', label='Humidity (%)', linewidth=2)
ax1.plot(time, ref_trajectory[:, 0], 'g--', label='Temperature ref (°C)', linewidth=2)
ax1.plot(time, ref_trajectory[:, 1], 'y--', label='Humidity ref (°C)', linewidth=2)


ax1.set_ylabel('Greenhouse States')
ax1.set_title('MPC Performance: Temperature & Humidity Tracking')
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)

# Plot 2: Control Input (Heater and humidifier)
ax2.step(time, u_trajectory[:,0], 'g-', where='post', label='Heater Power (%)', linewidth=2)
ax2.step(time, u_trajectory[:,1], 'y-', where='post', label='Humidifier Power (%)', linewidth=2)
ax2.set_ylabel('Input (%)')
ax2.set_xlabel('Time Steps')
ax2.set_ylim([0, 110]) 
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()


