import numpy as np

class BasePlant:
    def __init__(self, initial_state, A, B):
        # A is now 7x7, B is 7x2
        self.A = A
        self.B = B
        self.x = np.array(initial_state).reshape(-1, 1)
        self.u_prev = np.zeros((B.shape[1], 1)) 

    def update(self, u_cmd, solar_gain):
        # Update actuators (clamped to 0-100)
        self.u_prev = np.clip(u_cmd.reshape(-1, 1), 0.0, 100.0)
        
        # Disturbance: Pad 7x1 vector so solar gain only affects Temperature (index 0)
        disturbance = np.zeros((7, 1))
        disturbance[0, 0] = solar_gain
        
        # Physics: x(k+1) = Ax(k) + Bu(k) + Disturbance
        self.x = (self.A @ self.x) + (self.B @ self.u_prev) + disturbance
        
        return self.x.flatten()