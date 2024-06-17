import numpy as np
import math
from scipy.optimize import minimize, rosen, rosen_der


def cost_state_1(control_ver_0):
    curr_state = [0,0,0]
    x0 = curr_state[0]
    y0 = curr_state[1]
    r0 = curr_state[2]
    dt = 1
    v0 = control_ver_0[0]
    w0 = control_ver_0[1]
    
    A = np.array([ [1,0,0], [0,1,0], [0,0,1]])
    B = np.array([ [ math.cos(r0)*dt, 0], [math.sin(r0)*dt, 0], [0, dt]])
    
    predict_state_1 = np.dot(A, curr_state) + np.dot(B, control_ver_0)
    print(predict_state_1)
    error = (predict_state_1[0]-20)**2 + (predict_state_1[1]-0)**2 + (predict_state_1[2]-20)**2
    print("error", error)
    return error


res = minimize(cost_state_1, [20,20], method='Nelder-Mead', tol=1e-6)

print("optimize", res.x)

cost_state_1([1, 1])