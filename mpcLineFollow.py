

import math
import numpy as np

state_0 = np.array([ [0], [0], [0]])
weight = np.array([[1], [2], [3]])
start_state = state_0
end_state = np.array([[3], [3], [0]])

control_ver = np.array( [ [[1], [0]], [[1], [1]], [[1], [-1]]])




# predicted stste for dt = 0.1 seconds
def robot_model(curr_state, control_ver):
	x0 = curr_state[0]
	y0 = curr_state[1]
	r0 = curr_state[2]
	v0 = control_ver[0]
	w0 = control_ver[1]
	dt = 0.01

	A = np.array([ [1,0,0], [0,1,0], [0,0,1]])
	B = np.array([ [ math.cos(r0)*dt, 0], [math.sin(r0)*dt, 0], [0, dt]])

	predict_state = np.dot(A, curr_state) + np.dot(B, control_ver)

	return predict_state



# print( robot_model(start_state, control_ver))

# for i in range(10):
# 	predict =  robot_model(start_state, control_ver_0)
# 	start_state = predict

# print(predict)


def cost_fun():
	start_state = state_0
	ref_state = end_state
	value = np.array([ [0], [0], [0]])

	for step in range(3):		
	
		for i in range(100):
			predict =  robot_model(start_state, control_ver[step])
			start_state = predict
		# print("\ncontrol matrix at", step, "state", control_ver[step])
		# print("predict state", predict)

	return ref_state - predict
		
		

error = cost_fun()
print("\ncost value", error)



# error_r = 100

# while error_r < 0.01 :




