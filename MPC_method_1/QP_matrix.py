

# state var - x_k, y_k, theta_k(radian)
# control var - v_k, w_k

# state equations
# x_k+1 = x_k + v_k.cos(theta_k).dt
# y_k+1 = y_k + v_k.sin(theta_k).dt
# theta_k+1 = theta_k + w_k.dt

# linearize state equations
# x_k+1 = x_k  +  v_k.cos(theta_k).dt  -  v_k.sin(theta_k).dt.theta_k
# y_k+1 = y_k  +  v_k.sin(theta_k).dt  +  v_k.cos(theta_k).dt.theta_k
# theta_k+1 = theta_k + w_k.dt


import numpy as np
import math


init_state_val = np.array([ [1], [1], [0]])
init_control_val = np.array([ [1], [1]])

ref_state_val = np.array([[0, 0.1, 0.2, 0.3, 0.4, 0.5], 
                          [0, 0.1, 0.2, 0.3, 0.4, 0.5], 
                          [0,   0,   0,   0,   0,   0]])

pred_state_val = np.zeros(3,6)


def A_k( state_val, control_val, dt) :
    x_k = state_val[0][0]
    y_k = state_val[1][0]
    theta_k = state_val[2][0]
    v_k = control_val[0][0]
    w_k = control_val[1][0]

    return np.array( [ [1, 0, -(v_k*math.sin(theta_k)*dt) ], 
                       [0, 1,  (v_k*math.cos(theta_k)*dt) ], 
                       [0, 0,                          1  ]])

def B_k( state_val, control_val, dt) :
    x_k = state_val[0][0]
    y_k = state_val[1][0]
    theta_k = state_val[2][0]
    v_k = control_val[0][0]
    w_k = control_val[1][0]

    return np.array( [ [math.cos(theta_k)*dt, 0 ],
                       [math.sin(theta_k)*dt, 0 ],
                       [                   0, dt]])

A_0 = A_k( init_state_val, init_control_val, 0.01)
B_0 = B_k( init_state_val, init_control_val, 0.01)
X_1 = np.dot(A_0 , init_state_val) + np.dot(B_0 , init_control_val)

print(X_1)

