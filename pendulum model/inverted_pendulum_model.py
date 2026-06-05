import math


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class InvertedPendulumCart:
    def __init__(
        self,
        x=0.0,
        x_dot=0.0,
        theta=math.radians(12.0),
        theta_dot=0.0,
        cart_mass=1.0,
        pendulum_mass=0.2,
        pendulum_length=0.75,
        gravity=9.81,
        max_force=15.0,
    ):
        self.cart_mass = cart_mass
        self.pendulum_mass = pendulum_mass
        self.pendulum_length = pendulum_length
        self.gravity = gravity
        self.max_force = max_force
        self.start_state = (x, x_dot, theta, theta_dot)
        self.reset()

    def reset(self):
        self.x, self.x_dot, self.theta, self.theta_dot = self.start_state
        self.u = 0.0

    @property
    def state(self):
        return (self.x, self.x_dot, self.theta, self.theta_dot)

    def dynamics(self, state, control):
        x, x_dot, theta, theta_dot = state
        m = self.pendulum_mass
        M = self.cart_mass
        l = self.pendulum_length
        g = self.gravity

        sin_t = math.sin(theta)
        cos_t = math.cos(theta)
        denom = M + m * sin_t * sin_t

        x_ddot = (
            control
            + m * l * theta_dot * theta_dot * sin_t
            - m * g * sin_t * cos_t
        ) / denom
        theta_ddot = (
            -control * cos_t
            - m * l * theta_dot * theta_dot * sin_t * cos_t
            + (M + m) * g * sin_t
        ) / (l * denom)

        return x_dot, x_ddot, theta_dot, theta_ddot

    def step(self, control, dt):
        self.u = clamp(control, -self.max_force, self.max_force)
        state = self.state

        def derivative(s, u):
            return self.dynamics(s, u)

        k1 = derivative(state, self.u)
        s2 = tuple(state[i] + 0.5 * dt * k1[i] for i in range(4))
        k2 = derivative(s2, self.u)
        s3 = tuple(state[i] + 0.5 * dt * k2[i] for i in range(4))
        k3 = derivative(s3, self.u)
        s4 = tuple(state[i] + dt * k3[i] for i in range(4))
        k4 = derivative(s4, self.u)

        next_state = tuple(
            state[i] + dt * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0
            for i in range(4)
        )

        self.x, self.x_dot, self.theta, self.theta_dot = next_state
        self.theta = wrap_angle(self.theta)

    def reset_angle(self, degrees):
        self.theta = math.radians(degrees)
        self.theta_dot = 0.0
