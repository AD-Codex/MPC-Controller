import rospy
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np
import math
from scipy.optimize import minimize, rosen, rosen_der


# 1. method='trust-constr' -------------------------------------------------------
a = 0.2
# control_matrix = [a,0, a,0, a,0, a,0, a,0, a,0, a,0, a,0, a,0]
# follow_path = np.array([ [0, 5, 0], [0.5, 5, 0], [1, 5, 0], [1.5, 5, 0], [2, 5, 0], [2.5, 5, 0], [3, 5, 0], [3.5, 5, 0], [4, 5, 0], [4.5, 5, 0], [5, 5, 0]])
# weight = np.array([ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

# object_location = np.array([ 3, 5, 0])
# oblect_radius = 2
# weight_constrain = np.array([ 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5])
# factor = 0.6


a = 0.2
control_matrix = [a,0, a,0, a,0, a,0, a,0, a,0, a,0, a,0, a,0]
follow_path = np.array([ [0, 5, 0], [0.5, 5, 0], [1, 5, 0], [1.5, 5, 0], [2, 5, 0], [2.5, 5, 0], [3, 5, 0], [3.5, 5, 0], [4, 5, 0], [4.5, 5, 0], [5, 5, 0]])
weight = np.array([ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

object_location = np.array([ 3, 4, 0])
oblect_radius = 2
weight_constrain = np.array([ 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5])
factor = 0.6

prediacted_states = []

def cost_Fn( matrix):
    global prediacted_states
    global follow_path
    global weight

    control_matrix = []
    for i in range( 0, len(matrix), 2):
        control_matrix.append( [ matrix[i], matrix[i+1]])

    curr_state = [0,5,0]
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

    # cost fn with constrain
    print("constrain error")
    J_constrain = 0
    L = 2 # least distance
    for count, predict_state in enumerate(prediacted_states):
        distance_x = predict_state[0] - object_location[0]
        distance_y = predict_state[1] - object_location[1]
        orientation = predict_state[2]
        angle = math.atan(distance_y/distance_x)
        distanse = math.sqrt(distance_x**2 + distance_y**2)

        theta = angle + orientation
        error = abs(distanse * math.sin(theta)) - (L + oblect_radius)
        if distance_x < 0 and distanse > 3 and theta > 1 :
            print("no error", end=" ")
            J_constrain = J_constrain
        elif distance_x < 1 :
            print(error, end=" ")
            J_constrain = J_constrain + weight_constrain[count] * (error**2)
        else:
            print("no error", end=" ")
            J_constrain = J_constrain

    
    # cost fn with line follow
    print("\nline error")
    J = np.array([0,0,0])
    for count, predict_state in enumerate(prediacted_states):
        error = predict_state -  follow_path[count]
        print(error, end=" ")
        J = J + weight[count] * (error**2)

    # print(prediacted_states)
    Terror = factor*(J[0] + J[1] + J[2]) + (1-factor)*J_constrain
    print("\nerror" , Terror)

    return  Terror


res = minimize(cost_Fn, control_matrix, method='trust-constr', tol=1e-6)



marker = Marker()
markerArray = MarkerArray()


if __name__ == '__main__':
    rospy.init_node('road_point')
    rospy.loginfo("road_point node start")

    marker_pub = rospy.Publisher("/object", MarkerArray, queue_size = 10)

    rate = rospy.Rate(20)

    i = 0

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
            marker.scale.x = 0.2
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # Color
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5

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

            markerArray.markers.append(marker)



        
        for i in range( len(prediacted_states)):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = ""

            # Shape (mesh resource type - 10)
            marker.type = Marker.ARROW
            marker.id = 100+ i
            marker.action = Marker.ADD

            # Scale
            marker.scale.x = 0.2
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # Color
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5

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

            markerArray.markers.append(marker)

        
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ""

            # Shape (mesh resource type - 10)
        marker.type = Marker.CYLINDER
        marker.id = 1000
        marker.action = Marker.ADD

            # Scale
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

            # Color
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5

            # Pose
        marker.pose.position.x = object_location[0]
        marker.pose.position.y = object_location[1]
        marker.pose.position.z = 0.5

        markerArray.markers.append(marker)

        marker_pub.publish(markerArray)



        rate.sleep()

    rospy.spin()
