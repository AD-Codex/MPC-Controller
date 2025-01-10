import rospy
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np
import math
from scipy.optimize import minimize, rosen, rosen_der


# 1. method='Nelder-Mead' -------------------------------------------------------
# control_matrix = [0,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0]
# follow_path = np.array([ [0, -5, 0], [2, -4.8, 0], [4, -4.6, 0], [6, -4.4, 0], [8, -4.2, 0], [10, -4, 0], [12, -3.8, 0], [14, -3.6, 0], [16, -3.4, 0], [18, -3.2, 0], [20, -3, 0]])
# weight = np.array([ 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5])

# 2. method='Nelder-Mead' -------------------------------------------------------
# control_matrix = [0,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0]
# follow_path = np.array([ [0, -5, -0.1], [2, -4.8, -0.2], [4, -4.5, -0.3], [6, -4, -0.4], [8, -3.5, 0], [10, -3.5, 0], [12, -3.5, 0], [14, -3.6, 0], [16, -3.4, 0], [18, -3.2, 0], [20, -3, 0]])
# weight = np.array([ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

# 3. method='trust-constr'
# control_matrix = [0,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0]
# follow_path = np.array([ [0, -5, -0.1], [2, -4.8, -0.2], [4, -4.5, -0.3], [6, -4, -0.4], [8, -3.5, 0], [10, -3.5, 0], [12, -3.5, 0], [14, -3.6, 0], [16, -3.4, 0], [18, -3.2, 0], [20, -3, 0]])
# weight = np.array([ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])



control_matrix = [0,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0, 1,0]
follow_path = np.array([ [0, -5, -0.1], [2, -4.8, -0.2], [4, -4.5, -0.3], [6, -4, -0.4], [8, -3.5, 0], [10, -3.5, 0], [12, -3.5, 0], [14, -3.6, 0], [16, -3.4, 0], [18, -3.2, 0], [20, -3, 0]])
weight = np.array([ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
prediacted_states = []

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

    return J[0]**2 + J[1]**2 + J[2]**2


res = minimize(cost_Fn, control_matrix, method='Nelder-Mead', tol=1e-6)



marker = Marker()
markerArray1 = MarkerArray()
markerArray2 = MarkerArray()



if __name__ == '__main__':
    rospy.init_node('road_point')
    rospy.loginfo("road_point node start")

    marker1_pub = rospy.Publisher("/object/marker1", MarkerArray, queue_size = 10)
    marker2_pub = rospy.Publisher("/object/marker2", MarkerArray, queue_size = 10)

    rate = rospy.Rate(20)


    i= 0

    while not rospy.is_shutdown():

        for i in range( len(follow_path)):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = ""

            # Shape (mesh resource type - 10)
            marker.type = Marker.ARROW
            marker.id = i
            marker.action = Marker.ADD

            # Scale
            marker.scale.x = 1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Color
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Pose
            marker.pose.position.x = follow_path[i][0]
            marker.pose.position.y = follow_path[i][1]
            marker.pose.position.z = 0.5

            # matrix shift
            q_orig = quaternion_from_euler(0, 0, follow_path[i][2])
            q_rot = quaternion_from_euler(math.pi, 0, 0)
            q_new = quaternion_multiply(q_rot, q_orig)
            # print(q_new)

            marker.pose.orientation.x = q_new[0]
            marker.pose.orientation.y = q_new[1]
            marker.pose.orientation.z = q_new[2]
            marker.pose.orientation.w = q_new[3]

            markerArray1.markers.append(marker)



        
        for i in range( len(prediacted_states)):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = ""

            # Shape (mesh resource type - 10)
            marker.type = Marker.ARROW
            marker.id = i
            marker.action = Marker.ADD

            # Scale
            marker.scale.x = 1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Color
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Pose
            marker.pose.position.x = prediacted_states[i][0]
            marker.pose.position.y = prediacted_states[i][1]
            marker.pose.position.z = 0.5

            # matrix shift
            q_orig = quaternion_from_euler(0, 0, -prediacted_states[i][2])
            q_rot = quaternion_from_euler(math.pi, 0, 0)
            q_new = quaternion_multiply(q_rot, q_orig)
            # print(q_new)

            marker.pose.orientation.x = q_new[0]
            marker.pose.orientation.y = q_new[1]
            marker.pose.orientation.z = q_new[2]
            marker.pose.orientation.w = q_new[3]

            markerArray2.markers.append(marker)


        marker1_pub.publish(markerArray1)
        marker2_pub.publish(markerArray2)



        rate.sleep()

    rospy.spin()
