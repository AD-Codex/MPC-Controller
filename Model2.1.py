import numpy as np
import math


control_matrix = np.array([ [1,1], [1,1], [1,1], [1,1], [1,1]])
follow_path = np.array([ [0, 0, 0], [2, 1, 0], [4, 2, 0], [6, 3, 0], [8, 4, 0], [10, 5, 0]])
weight = np.array([ 1, 2, 3, 4, 5, 6])


def cost_Fn( control_matrix ):
    global prediacted_states
    global follow_path
    global weight

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

    return J[0] + J[1] + J[2]


print(cost_Fn(control_matrix))





