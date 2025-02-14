
# MPC Controller with customizable object adding
# multi Static object avoiding, long range
# tested with 2 objects
# correct method 


# ---------------------- state equations ----------------------------------------
#
#                       k+1 step
#                      .
#                   .
#                .
#             .
#          .
#       .
#     K step - (X_k, Y_k, theta_k) 
#  ( v_k, w_k)

# x_k+1 = x_k + v_k.cos(theta_k).dt
# y_k+1 = y_k + v_k.sin(theta_k).dt
# theta_k+1 = theta_k + w_k.dt

# | X_k+1     |   | 1  0  0 | | X_k     |     | cos(theta_k).dt   0 | | V_k |
# | Y_k+1     | = | 0  1  0 |.| Y_k     |  +  | sin(theta_k).dt   0 |.| w_k |
# | theta_k+1 |   | 0  0  1 | | theta_k |     |               0  dt | 


# ----------------------- consider a static object ---------------------------------
#
#                    static object - ( X_s, Y_s)
#                    ..
#                  . .
#                .  .
#              .   .
#            .    .
#     l_k  .     . l_k+1
#        .      . 
#      .        k+1 step - (X_k+1, Y_k+1, theta_k+1)             
#    . 
# K step - (X_k, Y_k, theta_k)      
#   (v_k, w_k)

# orientation between,     K+1 step & K step  =  theta_k
# angle to static object,       alpha_k       =  aniSin[ ly_k/l_k]
# ori & static object angle,                  =  alphs_k - theta_k
#                                     l_k+1   =  l_k  -  v_k. cos( alpha_k - theta_k) . dt    ; consider small dts
#
#                                     
# x_k+1     = x_k + v_k.cos(theta_k).dt
# y_k+1     = y_k + v_k.sin(theta_k).dt
# theta_k+1 = theta_k + w_k.dt
# l_k+1     = l_k  -  v_k.cos( alpha_k - theta_k).dt
# ly_k+1    = ly_k - v_k.sin(theta_k).dt

# | X_k+1     |   | 1  0  0  0  0 | | X_k     |     |            cos(theta_k).dt   0 | | V_k |
# | Y_k+1     | = | 0  1  0  0  0 |.| Y_k     |  +  |            sin(theta_k).dt   0 |.| w_k |
# | theta_k+1 |   | 0  0  1  0  0 | | theta_k |     |                          0  dt | 
# | l_k+1     | = | 0  0  0  1  0 | | l_k     |     |-cos( alpha_k - theta_k).dt   0 |
# | ly_k+1    | = | 0  0  0  0  1 | | ly_k    |     |           -sin(theta_k).dt   0 |


# ----------------- linearize state equations ------------------------------------
# x_k+1     = x_k     +  v_k.cos(theta_k).dt  -  v_k.sin(theta_k).dt.theta_k
# y_k+1     = y_k     +  v_k.sin(theta_k).dt  +  v_k.cos(theta_k).dt.theta_k
# theta_k+1 = theta_k +  w_k.dt
# l_k+1     = l_k     -  v_k.cos(alpha_k - theta_k).dt  - v_k.sin(alpha_k - theta_k).dt.theta_k - [v_k.sin(alpha_k-theta_k).dt].l_k/[cos(alpha_k).l_k.l_k] + [v_k.sin(alpha_k-theta_k).dt].ly_k/cos(alpha_k)
# ly_k+1    = ly_k    -  v_k.sin(theta_k).dt  -  v_k.cos(theta_k).dt.theta_k


# | X_k+1     |   | 1  0           -v_k.sin(theta_k).dt                                                            0                                                  0 | | X_k     |     |            cos(theta_k).dt   0 | | V_k |
# | Y_k+1     | = | 0  1            v_k.cos(theta_k).dt                                                            0                                                  0 |.| Y_k     |  +  |            sin(theta_k).dt   0 |.| w_k |
# | theta_k+1 |   | 0  0                              1                                                            0                                                  0 | | theta_k |     |                          0  dt | 
# | l_k+1     | = | 0  0  - v_k.sin(alpha_k-theta_k).dt  1-[v_k.sin(alpha_k-theta_k).ly_k.dt]/[cos(alpha_k).l_k.l_k]  [v_k.sin(alpha_k-theta_k).dt]/[cos(alpha_k).ly_k] | | l_k     |     |-cos( alpha_k - theta_k).dt   0 |
# | ly_k+1    | = | 0  0           -v_k.cos(theta_k).dt                                                            0                                                  1 | | ly_k    |     |           -sin(theta_k).dt   0 |

#     X_1    =           A_0                 . X_0         +          B_0            . U_0
#     X_2    =           A_1                 . X_1         +          B_1            . U_1


# ----------------------- STATE MATRIX ------------------------------------------
# X_predict = phi . x_0 + tau .U_predict  

# X_predict = [ x_1, y_1, theta_1, l_1, ly_1, x_2, y_2, theta_2, l_2, ly_2 x_3, y_3, theta_3, l_3, ly_3 x_4, y_4, theta_4 l_4, ly_4] ^ T

# phi = [          A0]
#       [       A1.A0]
#       [    A2.A1.A0]
#       [ A3.A2.A1.A0]

# tau = [          B0,         0,       0,      0]
#       [       A1.B0,        B1,       0,      0]    
#       [    A2.A1.B0,     A2.B1,      B2,      0]  
#       [ A3.A2.A1.B0,  A3.A2.B1,   A3.B2,     B3]


# --------------------------- Cost Fn --------------------------------------
# J = (X_predict - X_ref)^T . Q . (X_predict - X_ref) + U_predict^T . R . U_predict
#   = (1/2) . U_predict^T . H . U_predict + f^T . U_predict + constant

# H = tau^T . Q . tau + R
# f = tau^T . Q . ( phi . x_0 - X_ref)


# -------------------------- daqp general method ------------------------------
# (xstar,fval,exitflag,info) = daqp.solve(H,f,A,bupper,blower,sense)



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
X_0 = np.array([[0], [0], [0]])
initCoord = X_0

# object coordinates
staticObj = np.array([[  0.5,  0.8],
                      [    0,  0.1],
                      [    0,    0] ])

# starting point to object distane
for i in range( len(staticObj[0])):
    staticObj[2][i] = ( (staticObj[0][i] - X_0[0][0])**2 + (staticObj[1][i] - X_0[1][0])**2 )**(1/2)
    # initialize coordinate of system
    initCoord = np.vstack( (initCoord, np.array([ [staticObj[2][i]], [ staticObj[1][i] - X_0[1][0]] ]) ))


# predicted control states (number)
pred_control_val = np.tile( [[1],[0]], 20)
# cost Fn control state constant
control_val_R = np.zeros( len(pred_control_val[0])*2)

# step time seconds
dt = 0.025

# reference values, (numer + 1)
ref_state_val = np.array([[0, 0.05,  0.1, 0.15,  0.2, 0.25,  0.3, 0.35,  0.4, 0.45,  0.5, 0.55,  0.6, 0.65,  0.7, 0.75,  0.8, 0.85,  0.9, 0.95,    1], 
                          [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0],
                          [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0] ])



# cost fn state value constant (number)
state_val_Q = np.array([ [    1,    1,    2,    2,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8,    8,    9,    9,   10,   10],
                         [    1,    1,    2,    2,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8,    8,    9,    9,   10,   10],
                         [ 0.05, 0.05, 0.10, 0.10, 0.15, 0.15, 0.20, 0.20, 0.25, 0.25, 0.30, 0.30, 0.35, 0.35, 0.40, 0.40, 0.45, 0.45, 0.50, 0.50] ])


for i in range(states - 3):
    ref_state_val = np.vstack(( ref_state_val, np.zeros(21)))
    state_val_Q = np.vstack(( state_val_Q, np.zeros(20)))

# update ref_state_val and state_val_Q -----------------------------
space_val = [ 0.1, 0.1]
steps = 2
weight = 100
for objs in range( len(staticObj[0])):
    for i in range( len(ref_state_val[0])):
        if ( (staticObj[0][objs] - X_0[0][0]) - ref_state_val[0][i] < 0.01 ):
            for j in range(steps+1):
                # if ly_ref positive (+0.1) path goes under, else ly_ref negative (-0.1) path goes above
                ref_state_val[ 2*(objs+1) + 1 ][i - j] = space_val[objs]
                ref_state_val[ 2*(objs+1) + 1 ][i + j] = space_val[objs]
                ref_state_val[ 2*(objs+1) + 2 ][i - j] = space_val[objs] + j*0.02
                ref_state_val[ 2*(objs+1) + 2 ][i + j] = space_val[objs] + j*0.02

                state_val_Q[0][i-j] = weight
                state_val_Q[0][i-j] = weight

                state_val_Q[ 2*(objs+1) + 1 ][i+j] = weight
                state_val_Q[ 2*(objs+1) + 1 ][i-j] = weight
                state_val_Q[ 2*(objs+1) + 2 ][i+j] = weight
                state_val_Q[ 2*(objs+1) + 2 ][i-j] = weight
            break

print(state_val_Q)
state_val_Q = state_val_Q.flatten(order='F')
state_val_Q = np.diag(state_val_Q)



time.sleep(0.01)



while (True):
    control_val, state_value= QPC.QP_solutions( states, initCoord, pred_control_val, ref_state_val, control_val_R, state_val_Q, dt)
    if ( np.isnan(control_val[0][0])) :
        print(control_val[0][0], type(control_val[0][0]))
        time.sleep(0.01)
    else :
        break


refMarkerArray     = MarkerArray()
predictMarkerArray = MarkerArray()
staticObjArray     = MarkerArray()



# referance coord marker array ---------------------
def referance_markers( ref_val):
    global refMarkerArray

    for i in range( len(ref_val[0])):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ""

        # Shape (mesh resource type - 10)
        marker.type = Marker.ARROW
        marker.id = i
        marker.action = Marker.ADD

        # Scale
        marker.scale.x = 0.2
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        # Color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Pose
        marker.pose.position.x = ref_val[0][i] * 10
        marker.pose.position.y = ref_val[1][i] * 10
        marker.pose.position.z = 0.4

        # matrix shift
        q_orig = quaternion_from_euler(0, 0, -ref_val[2][i])
        q_rot = quaternion_from_euler(math.pi, 0, 0)
        q_new = quaternion_multiply(q_rot, q_orig)
        # print(q_new)

        marker.pose.orientation.x = q_new[0]
        marker.pose.orientation.y = q_new[1]
        marker.pose.orientation.z = q_new[2]
        marker.pose.orientation.w = q_new[3]

        refMarkerArray.markers.append(marker)

    return refMarkerArray


# predicted coord marker array ----------------------
def predicted_markers( ref_val):
    global predictMarkerArray

    for i in range( len(ref_val[0])):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ""

        # Shape (mesh resource type - 10)
        marker.type = Marker.ARROW
        marker.id = 1000 + i
        marker.action = Marker.ADD

        # Scale
        marker.scale.x = 0.2
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        # Color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = ( 1.0 - (i/len(ref_val[0])) )

        # Pose
        marker.pose.position.x = ref_val[0][i] *10
        marker.pose.position.y = ref_val[1][i] *10
        marker.pose.position.z = 0.5

        # matrix shift
        q_orig = quaternion_from_euler(0, 0, -ref_val[2][i])
        q_rot = quaternion_from_euler(math.pi, 0, 0)
        q_new = quaternion_multiply(q_rot, q_orig)
        # print(q_new)

        marker.pose.orientation.x = q_new[0]
        marker.pose.orientation.y = q_new[1]
        marker.pose.orientation.z = q_new[2]
        marker.pose.orientation.w = q_new[3]

        predictMarkerArray.markers.append(marker)

    return predictMarkerArray


# object marker array ------------------------------
def staticObj_marker( staticObj):
    global staticObjArray

    for i in range( len(staticObj[0])):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ""

        # Shape (mesh resource type - 10)
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.id = 4000 + i

        # Scale
        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 1

        # Color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        # Pose
        marker.pose.position.x = staticObj[0][i] *10
        marker.pose.position.y = staticObj[1][i] *10
        marker.pose.position.z = 0.5

        staticObjArray.markers.append(marker)

    return staticObjArray



if __name__ == '__main__':
    rospy.init_node('MPC_QP_method')
    rospy.loginfo("node start")

    refMarker_pub = rospy.Publisher("/object/refernce_point", MarkerArray, queue_size = 10)
    predictMarker_pub = rospy.Publisher("/object/predict_point", MarkerArray, queue_size = 10)
    staticObj_pub = rospy.Publisher("/object/static_obj", MarkerArray, queue_size = 10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        
        refMarker_pub.publish( referance_markers(ref_state_val))
        predictMarker_pub.publish( predicted_markers(state_value))
        staticObj_pub.publish( staticObj_marker( staticObj))


        rate.sleep()

    rospy.spin()