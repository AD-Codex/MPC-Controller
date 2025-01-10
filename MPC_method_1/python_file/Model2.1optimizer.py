import numpy as np
import math
from scipy.optimize import minimize, rosen, rosen_der


control_matrix = [1,1, 1,1, 1,1, 1,1, 1,1, 1,1, 1,1, 1,1, 1,1]
follow_path = np.array([ [0, 0, 0], [2, 1, 0], [4, 2, 0], [6, 3, 0], [8, 4, 0], [10, 5, 0], [12, 5, 0], [14, 5, 0], [16, 5, 0], [18, 5, 0], [20, 5, 0]])
weight = np.array([ 10, 9, 8, 7, 6, 5, 4, 3, 2, 1])
state_weight = np.array([3,2,1])

def cost_Fn( matrix):
    global prediacted_states
    global follow_path
    global weight

    control_matrix = []
    for i in range( 0, len(matrix), 2):
        control_matrix.append( [ matrix[i], matrix[i+1]])

    curr_state = [0,0,0]
    prediacted_states = np.array([ curr_state ])
    dt = 1

    for control_veriable in control_matrix :
        x0 = curr_state[0]
        y0 = curr_state[1]
        r0 = curr_state[2]
        v0 = control_veriable[0]
        w0 = control_veriable[1]

        A = np.array([ [1,0,0], [0,1,0], [0,0,1]])
        B = np.array([ [ math.cos(r0)*dt, 0], [math.sin(r0)*dt, 0], [0, dt]])

        predict_state = np.dot(A, curr_state) + np.dot(B, control_veriable)
        prediacted_states = np.vstack([prediacted_states, predict_state])

        curr_state = predict_state

    J = np.array([0,0,0])
    for count, predict_state in enumerate(prediacted_states) :
        J = J + weight[count] * ( predict_state -  follow_path[count]) ** 2

    print(prediacted_states)
    print("error" , J)

    return state_weight[0]*J[0] + state_weight[1]*J[1] + state_weight[2]*J[2]




res = minimize(cost_Fn, control_matrix, method='Nelder-Mead', tol=1e-2)

print("----------------------------------------------")
print("optimize", res.x)




