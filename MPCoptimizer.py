import numpy as np
from qpsolvers import solve_qp

class MPCoptimizer:
    def __init__(self, system, R, Q, hz, u_min=None, u_max=None):
        self.system = system
        self.A = system.A # Physical A (2x2)
        self.B = system.B # Physical B (2x2)
        self.hz = hz
        self.n, self.m = self.B.shape
        self.u_min = np.zeros(self.m) if u_min is None else np.array(u_min, dtype=float)
        self.u_max = np.full(self.m, 100.0) if u_max is None else np.array(u_max, dtype=float)
        
        # Create Augmented Matrices (4x4 and 4x2)
        # A_aug_system: [[A, B], [0, I]]
        A_top = np.hstack([self.A, self.B])
        A_bot = np.hstack([np.zeros((self.m, self.n)), np.eye(self.m)])
        A_aug_system = np.vstack([A_top, A_bot])
        
        # B_aug_system: [B; I]
        B_aug_system = np.vstack([self.B, np.eye(self.m)])

        # These are the prediction matrices over the horizon
        self.A_pred, self.B_pred = self.augmentation(A_aug_system, B_aug_system)

        # Selection Matrix C: Picks [T, H] out of [T, H, u1, u2]
        self.C_select = np.hstack([np.eye(self.n), np.zeros((self.n, self.m))])
        # Expand C for the whole horizon
        self.C_total = np.kron(np.eye(self.hz), self.C_select)

        self.Q_large = np.kron(np.eye(self.hz), Q)
        self.R_large = np.kron(np.eye(self.hz), R)

        # Updated Hessian: Includes the C_total filter
        CB = self.C_total @ self.B_pred
        state_term = CB.T @ self.Q_large @ CB
        self.H = 2 * (state_term + self.R_large)

    def augmentation(self, A, B):
        n_aug = A.shape[0]
        m_aug = B.shape[1]
        
        A_pred = np.zeros((self.hz * n_aug, n_aug))
        A_pow = np.eye(n_aug)
        basis_col = []
        
        for i in range(self.hz):
            A_pow = A_pow @ A
            A_pred[i*n_aug : (i+1)*n_aug, :] = A_pow
            basis_col.append(A_pow @ B if i > 0 else B)

        B_pred = np.zeros((self.hz * n_aug, self.hz * m_aug))
        for j in range(self.hz):
            for i in range(self.hz - j):
                B_pred[(j+i)*n_aug : (j+i+1)*n_aug, j*m_aug : (j+1)*m_aug] = basis_col[i]
                
        return A_pred, B_pred

    def solve(self, x_est, target_stack):
        # Augmented state X_k = [x_k, u_{k-1}]
        X_k = np.concatenate([x_est, self.system.u_prev.flatten()]).reshape(-1, 1)
        
        # Predicted states over horizon
        free_response = self.A_pred @ X_k
        
        # Apply C filter to compare ONLY physical states to targets
        error_vector = (self.C_total @ free_response) - target_stack
        
        CB = self.C_total @ self.B_pred
        f = 2 * CB.T @ self.Q_large @ error_vector

        # Slew rate bounds on delta_u; absolute limits are enforced by the plant clamp
        u0 = self.system.u_prev.flatten()
        lb = np.maximum(-2.0, np.tile(self.u_min - u0, self.hz))
        ub = np.minimum( 2.0, np.tile(self.u_max - u0, self.hz))

        delta_u_opt = solve_qp(self.H, f, lb=lb, ub=ub, solver="osqp",
                               polish=True, eps_abs=1e-7, eps_rel=1e-7)
        if delta_u_opt is None:
            delta_u_opt = np.zeros(self.hz * self.m)
        return delta_u_opt[0:self.m]