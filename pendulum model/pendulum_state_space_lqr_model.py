import numpy as np


def solve_discrete_lqr(a_d, b_d, q, r, iterations=120):
    p = q.copy()
    for _ in range(iterations):
        s = r + b_d.T @ p @ b_d
        k = np.linalg.solve(s, b_d.T @ p @ a_d)
        p = a_d.T @ p @ a_d - a_d.T @ p @ b_d @ k + q
    s = r + b_d.T @ p @ b_d
    return np.linalg.solve(s, b_d.T @ p @ a_d)


class InvertedPendulumLQRController:
    def __init__(self, cart_mass=1.0, pendulum_mass=0.2, pendulum_length=0.75, gravity=9.81):
        self.M = cart_mass
        self.m = pendulum_mass
        self.l = pendulum_length
        self.g = gravity
        self.Q = np.diag([10.0, 1.0, 200.0, 10.0])
        self.R = np.array([[0.5]])
        self.K = np.zeros((1, 4))

    def set_weights(self, q_x, q_xdot, q_theta, q_thetadot, r_u):
        self.Q = np.diag([q_x, q_xdot, q_theta, q_thetadot])
        self.R = np.array([[r_u]])

    def linearized_matrices(self):
        M = self.M
        m = self.m
        l = self.l
        g = self.g

        a = np.zeros((4, 4), dtype=float)
        a[0, 1] = 1.0
        a[1, 2] = -(m * g) / M
        a[2, 3] = 1.0
        a[3, 2] = ((M + m) * g) / (M * l)

        b = np.zeros((4, 1), dtype=float)
        b[1, 0] = 1.0 / M
        b[3, 0] = -1.0 / (M * l)

        return a, b

    def discretize(self, dt):
        a_c, b_c = self.linearized_matrices()
        a_d = np.eye(4) + a_c * dt
        b_d = b_c * dt
        return a_d, b_d

    def compute_gain(self, dt):
        a_d, b_d = self.discretize(dt)
        self.K = solve_discrete_lqr(a_d, b_d, self.Q, self.R)
        return self.K

    def control(self, state, x_ref=0.0):
        x, x_dot, theta, theta_dot = state
        error = np.array([x - x_ref, x_dot, theta, theta_dot], dtype=float)
        return float((-self.K @ error).item())
