import numpy as np

class KalmanFilter:
    def __init__(self, A, B, C, Q, R, initial_state):
        self.A = A
        self.B = B
        self.C = C
        self.Q = Q # Process noise
        self.R = R # Measurement noise
        self.x_hat = initial_state.reshape(-1, 1)
        self.P = np.eye(len(initial_state)) # Initial uncertainty

    def predict(self, u_prev):
        # state prediction x_hat = A*x + B*u
        self.x_hat = (self.A @ self.x_hat) + (self.B @ u_prev)
        # covariance prediction
        self.P = (self.A @ self.P @ self.A.T) + self.Q

    def update(self, y_measured):
        # Calculate Kalman Gain
        y_measured = y_measured.reshape(2, 1)
        S = (self.C @ self.P @ self.C.T) + self.R
        K = self.P @ self.C.T @ np.linalg.inv(S)
        
        # Correct the estimate with the measurement
        innovation = y_measured - (self.C @ self.x_hat)
        self.x_hat = self.x_hat + (K @ innovation)
        
        # Update uncertainty
        self.P = (np.eye(len(self.x_hat)) - (K @ self.C)) @ self.P
        return self.x_hat.flatten()