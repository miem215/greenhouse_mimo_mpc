import numpy as np

class BasePlant:
    def __init__(self, initial_state, A, B):
        self.A = A
        self.B = B
        self.x = initial_state
        # Maintain absolute power internally
        self.u_prev = np.zeros((B.shape[1], 1)) 

    def update(self, delt_u):
        # 1. Update absolute power: u(k) = u(k-1) + delta_u
        self.u_prev = self.u_prev + delt_u.reshape(-1, 1)
        
        # 2. Compute physics: x(k+1) = Ax(k) + Bu(k)
        self.x = (self.A @ self.x.reshape(-1, 1)) + (self.B @ self.u_prev)
        
        return self.x.flatten()