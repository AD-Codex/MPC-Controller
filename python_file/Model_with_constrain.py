import numpy as np
import math
from scipy.optimize import minimize, rosen, rosen_der


control_matrix = [1,1, 1,1, 1,1, 1,1, 1,1, 1,1, 1,1, 1,1, 1,1]
follow_path = np.array([ [0, 0, 0], [2, 1, 0], [4, 2, 0], [6, 3, 0], [8, 4, 0], [10, 5, 0], [12, 5, 0], [14, 5, 0], [16, 5, 0], [18, 5, 0], [20, 5, 0]])
weight = np.array([ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

object_location = np.array([ 3, 5])
weight_constrain = np.array([ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
factor = 0.5


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

    
    J1 = 0
    L = 2 # least distance
    for count, predict_state in enumerate(prediacted_states):
        distance_x = predict_state[0] - object_location[0]
        distance_y = predict_state[1] - object_location[1]
        J1 = J1 + weight_constrain[count] *( L -  math.sqrt(distance_x**2 + distance_y**2))


    J = np.array([0,0,0])
    for count, predict_state in enumerate(prediacted_states) :
        J = J + weight[count] * ( predict_state -  follow_path[count]) ** 2

    print(prediacted_states)
    print("error" , J)

    return (factor*(J[0]+J[1]+J[2]) + (1-factor)*J1 )




res = minimize(cost_Fn, control_matrix, method='Nelder-Mead', tol=1e-6)

print("----------------------------------------------")
print("optimize", res.x)




