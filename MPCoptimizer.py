import numpy as np
from qpsolvers import solve_qp

class MPCoptimizer:
    def __init__(self, A_aug, B_aug, C_aug, R, Q, hz):
        self.A = A_aug 
        self.B = B_aug 
        self.C = C_aug 
        self.R = R  # Control cost matrix (2x2)
        self.Q = Q  # State cost matrix (4x4)
        self.hz = hz
        self.n, self.m = 7, 2
        
        # Build 7-state prediction matrices
        self.A_pred = np.zeros((self.hz * self.n, self.n))
        self.B_pred = np.zeros((self.hz * self.n, self.hz * self.m))
        
        A_pow = np.eye(self.n)
        for i in range(self.hz):
            A_pow = A_pow @ self.A
            self.A_pred[i*self.n:(i+1)*self.n, :] = A_pow
            for j in range(i + 1):
                self.B_pred[i*self.n:(i+1)*self.n, j*self.m:(j+1)*self.m] = np.linalg.matrix_power(self.A, i-j) @ self.B

    def solve(self, x_est, target_stack):
        """
        Solves: min ||(C*B_pred)U + (C*A_pred)x_est - target||^2_Q + ||U||^2_R
        """
        C_large = np.kron(np.eye(self.hz), self.C)
        
        # System dynamics: Y = G*U + F*x_est
        G = C_large @ self.B_pred
        F = C_large @ self.A_pred
        
        # QP Cost Matrices: 0.5 * U.T * P * U + q.T * U
        Q_large = np.kron(np.eye(self.hz), self.Q)
        R_large = np.kron(np.eye(self.hz), self.R)
        
        P = G.T @ Q_large @ G + R_large
        q = G.T @ Q_large @ (F @ x_est - target_stack)
        
        # Constraints: 0 <= u <= 100
        lb = np.zeros(self.m * self.hz)
        ub = np.ones(self.m * self.hz) * 100.0
        
        # Solve
        u_opt = solve_qp(P, q, lb=lb, ub=ub, solver='osqp')
        
        return u_opt.reshape(-1, 1)