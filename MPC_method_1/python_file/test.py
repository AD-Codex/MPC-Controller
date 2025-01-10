import cvxpy as cp
import numpy as np

# Define control matrix size
n_controls = 5

# System parameters (assuming constant velocity model)
dt = 1  # Time step
state_weight = np.array([3, 2, 1])
weight = np.array([10, 9, 8, 7, 6, 5, 4, 3, 2, 1])
follow_path = np.array([
    [0, 0, 0], [2, 1, 0], [4, 2, 0], [6, 3, 0], [8, 4, 0],
    [10, 5, 0], [12, 5, 0], [14, 5, 0], [16, 5, 0], [18, 5, 0],
    [20, 5, 0]
])

# Define CVXPY variables
u = cp.Variable((2, n_controls))  # Control inputs (velocity, omega)
x = cp.Variable((3, n_controls + 1))  # States (x, y, theta)

# Initial state
x[:, 0] = cp.Constant(np.array([0, 0, 0]))  # Initial position and orientation

# Define system dynamics
def dynamics(x_prev, u):
    A = cp.Constant(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
    B = cp.Constant(np.array([[dt * cp.cos(x_prev[2]), 0], [dt * cp.sin(x_prev[2]), 0], [0, dt]]))
    return x_prev + A @ u

# Define constraints (all states must be within follow path boundaries)
constraints = []
for i in range(1, n_controls + 1):
    x_i = dynamics(x[:, i - 1], u)
    x[:, i] = x_i
    constraints.append(cp.abs(x[0, i] - follow_path[i - 1, 0]) <= 1e-3)
    constraints.append(cp.abs(x[1, i] - follow_path[i - 1, 1]) <= 1e-3)
    constraints.append(cp.abs(x[2, i] - follow_path[i - 1, 2]) <= 1e-3)

# Define cost function
cost = 0
for i in range(n_controls):
    # Tracking error
    tracking_error = cp.quad_form(x[:, i] - follow_path[i], weight[i])
    # Control effort penalty
    control_effort = cp.quad_form(u[:, i], np.eye(2))
    cost += tracking_error + state_weight @ control_effort

# Solve the optimization problem
problem = cp.Problem(cp.Minimize(cost), constraints)
result = problem.solve()

# Print optimized control inputs
print("Optimized control inputs:")
print(u.value.T)
