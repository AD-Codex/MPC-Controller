

import rospy
import numpy as np
from tf.transformations import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import QPC as QPC
import Frame_convert as Fc
import math
import follow_path as FP
import time
import sys



np.set_printoptions(threshold=sys.maxsize)


# number of states
states = 7

# starting point
X_0 = np.array([[0], [0.05], [0]])
initCoord = X_0

# object coordinates
staticObj = np.array([[  0.8,  1.3],
                      [ 0.05, 0.05],
                      [    0,    0] ])

# starting point to object distane
for i in range( len(staticObj[0])):
    staticObj[2][i] = ( (staticObj[0][i] - X_0[0][0])**2 + (staticObj[1][i] - X_0[1][0])**2 )**(1/2)
    # initialize coordinate of system
    initCoord = np.vstack( (initCoord, np.array([ [staticObj[2][i]], [ staticObj[1][i] - X_0[1][0]] ]) ))


# predicted control states (number)
pred_control_val = np.tile( [[1],[0]], 30)
# cost Fn control state constant
control_val_R = np.zeros( len(pred_control_val[0])*2)

# step time seconds
dt = 0.025

# reference values, (numer + 1)
ref_state_val = np.array([[0, 0.05,  0.1, 0.15,  0.2, 0.25,  0.3, 0.35,  0.4, 0.45,  0.5, 0.55,  0.6, 0.65,  0.7, 0.75,  0.8, 0.85,  0.9, 0.95,    1, 1.05,  1.1, 1.15,  1.2, 1.25,  1.3, 1.35,  1.4, 1.45,  1.5], 
                          [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0], 
                          [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                          [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                          [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                          [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                          [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]])

# cost fn state value constant (number)
state_val_Q = np.array([ [    1,    1,    2,    2,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8,    8,    9,    9,   10,   10,   11,   11,   12,   12,   13,   13,   14,   14,   15,   15],
                         [    1,    1,    2,    2,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8,    8,    9,    9,   10,   10,   11,   11,   12,   12,   13,   13,   14,   14,   15,   15],
                         [ 0.05, 0.05, 0.10, 0.10, 0.15, 0.15, 0.20, 0.20, 0.25, 0.25, 0.30, 0.30, 0.35, 0.35, 0.40, 0.40, 0.45, 0.45, 0.50, 0.50, 0.55, 0.55, 0.60, 0.60, 0.65, 0.65, 0.70, 0.70, 0.75, 0.75],
                         [    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                         [    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                         [    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                         [    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0] ])


# # update ref_state_val and state_val_Q -----------------------------
# space_val = [ 0.1, 0.1]
# steps = 2
# weight = 100
# for objs in range( len(staticObj[0])):
#     for i in range( len(ref_state_val[0])):
#         if ( staticObj[0][objs] - X_0[0][0] - ref_state_val[0][i] < 0.01 ):
#             for j in range(steps+1):
#                 # if ly_ref positive (+0.1) path goes under, else ly_ref negative (-0.1) path goes above
#                 ref_state_val[ 2*(objs+1) + 1 ][i - j] = space_val[objs]
#                 ref_state_val[ 2*(objs+1) + 1 ][i + j] = space_val[objs]
#                 ref_state_val[ 2*(objs+1) + 2 ][i - j] = space_val[objs] + j*0.02
#                 ref_state_val[ 2*(objs+1) + 2 ][i + j] = space_val[objs] + j*0.02

#                 state_val_Q[0][i-j] = weight
#                 state_val_Q[0][i-j] = weight

#                 state_val_Q[ 2*(objs+1) + 1 ][i+j] = weight
#                 state_val_Q[ 2*(objs+1) + 1 ][i-j] = weight
#                 state_val_Q[ 2*(objs+1) + 2 ][i+j] = weight
#                 state_val_Q[ 2*(objs+1) + 2 ][i-j] = weight
#             break



print("ref_state_val", ref_state_val)
print(state_val_Q)

state_val_Q = state_val_Q.flatten(order='F')
state_val_Q = np.diag(state_val_Q)

time.sleep(0.01)



while (True):
    control_val, state_value= QPC.QP_solutions( states, initCoord, pred_control_val, ref_state_val, control_val_R, state_val_Q, dt)
    if ( np.isnan(control_val[0][0])) :
        print(control_val[0][0], type(control_val[0][0]))
    else :
        break


refMarkerArray     = MarkerArray()
predictMarkerArray = MarkerArray()
staticObjArray     = MarkerArray()


