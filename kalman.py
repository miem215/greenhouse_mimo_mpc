import numpy as np

class KalmanFilter:
    def __init__(self, A, B, C, Q, R, initial_state):
        self.A = A # 7x7
        self.B = B # 7x2
        self.C = C # 4x7
        self.Q = Q # 7x7 process noise
        self.R = R # 4x4 measurement noise
        self.x_hat = np.array(initial_state).reshape(-1, 1)
        self.P = np.eye(7) 

    def predict(self, u_prev):
        self.x_hat = (self.A @ self.x_hat) + (self.B @ u_prev)
        self.P = (self.A @ self.P @ self.A.T) + self.Q

    def update(self, y_measured):
        # y_measured (4x1)
        S = (self.C @ self.P @ self.C.T) + self.R
        K = self.P @ self.C.T @ np.linalg.inv(S)
        
        innovation = y_measured.reshape(4, 1) - (self.C @ self.x_hat)
        self.x_hat = self.x_hat + (K @ innovation)
        self.P = (np.eye(7) - K @ self.C) @ self.P