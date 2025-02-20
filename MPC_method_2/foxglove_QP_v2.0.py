
# MPC controller
# convert frame to robot frame

# referance path get from follow_path.py


# -------------------------- convert to rbot frame coords --------------------
# 0 = robot orientation
# rotation_matrix = |  cos( 0), sin( 0) |
#                   | -sin( 0), cos( 0) |

# -------------------------- MPC controller ----------------------------
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

# | X_k+1     |   | 1  0  -v_k.sin(theta_k).dt | | X_k     |     | cos(theta_k).dt   0 | | V_k |
# | Y_k+1     | = | 0  1   v_k.cos(theta_k).dt |.| Y_k     |  +  | sin(theta_k).dt   0 |.| w_k |
# | theta_k+1 |   | 0  0                     1 | | theta_k |     |               0  dt |

# X_predict = phi . x_0 + tau .U_predict  -------- STATE MATRIX

# X_predict = [ x_1, y_1, theta_1, x_2, y_2, theta_2, x_3, y_3, theta_3, x_4, y_4, theta_4] ^ T

# phi = [          A0]
#       [       A1.A0]
#       [    A2.A1.A0]
#       [ A3.A2.A1.A0]

# tau = [          B0,         0,       0,      0]
#       [       A1.B0,        B1,       0,      0]    
#       [    A2.A1.B0,     A2.B1,      B2,      0]  
#       [ A3.A2.A1.B0,  A3.A2.B1,   A3.B2,     B3]

# U_predict = [ v0, w0, v1, w1, v2, w2, v3, w3, v4, w4] ^ T

# J = (X_predict - X_ref)^T . Q . (X_predict - X_ref) + U_predict^T . R . U_predict
#   = (1/2) . U_predict^T . H . U_predict + f^T . U_predict + constant

# H = tau^T . Q . tau + R
# f = tau^T . Q . ( phi . x_0 - X_ref)


import rospy
import numpy as np
from tf.transformations import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import QP_matrix as QPFn
import Frame_convert as Fc
import math
import follow_path as FP


X_0 = np.array([ [0], [0.5], [0]])
dt = 0.05

# reference state values [[ x],[ y],[ z]]
ref_state_val = np.array([[0, 0.05,  0.1, 0.15,  0.2, 0.25,  0.3, 0.35,  0.4, 0.45,  0.5, 0.55,  0.6, 0.65,  0.7, 0.75], 
                          [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0], 
                          [0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]])

# U_predict [ [v], [w]]
pred_control_val = np.array([[ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])


# control_val_R = np.zeros( len(pred_control_val[0])*2)
control_val_R = np.identity( len(pred_control_val[0])*2) *0.05

state_val_Q = np.array([1,1,0.05, 5,5,0.05, 5,5,0.1, 10,10,0.1, 10,10,0.15, 15,15,0.15, 15,15,0.2, 20,20,0.2, 20,20,0.25, 25,25,0.25, 25,25,0.3, 30,30,0.3, 30,30,0.35, 35,35,0.35, 40,40,0.4])
state_val_Q = np.diag(state_val_Q)


refMarkerArray = MarkerArray()
predictMarkerArray =MarkerArray()

path_coords = FP.path_coordinate()
inti_state, ref_state_val = Fc.Convert_To_Robot_Frame( X_0, path_coords)
# print(ref_state_val)

while (True):
    control_val, state_value= QPFn.QP_solutions( inti_state, dt, pred_control_val, ref_state_val, control_val_R, state_val_Q)
    if ( np.isnan(control_val[0][0])) :
        print(control_val[0][0], type(control_val[0][0]))
    else :
        break


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
        marker.color.a = 1.0

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



if __name__ == '__main__':
    rospy.init_node('MPC_QP_method')
    rospy.loginfo("node start")

    refMarker_pub = rospy.Publisher("/object/refernce_point", MarkerArray, queue_size = 10)
    predictMarker_pub = rospy.Publisher("/object/predict_point", MarkerArray, queue_size = 10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        
        refMarker_pub.publish( referance_markers(ref_state_val))

        while (True):
            control_val, state_value= QPFn.QP_solutions( inti_state, dt, pred_control_val, ref_state_val, control_val_R, state_val_Q)
            if ( np.isnan(control_val[0][0])) :
                print(control_val[0][0], type(control_val[0][0]))
            else :
                break
        predictMarker_pub.publish( predicted_markers(state_value))

        rate.sleep()

    rospy.spin()
