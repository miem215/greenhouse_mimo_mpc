import numpy as np

class BasePlant:
    def __init__(self, initial_state, A, B):
        self.A = A
        self.B = B
        self.x = initial_state
        self.u_prev = np.zeros((B.shape[1], 1)) 

    def update(self, delt_u, solar_gain):
        # Update absolute power: u(k) = u(k-1) + delta_u
        self.u_prev = self.u_prev + delt_u.reshape(-1, 1)
        
        # Compute physics: x(k+1) = Ax(k) + Bu(k) + solar disturbance
        disturbance = np.array([[solar_gain], [0.0]])
        self.x = (self.A @ self.x.reshape(-1, 1)) + (self.B @ self.u_prev)+disturbance
        
        return self.x.flatten()