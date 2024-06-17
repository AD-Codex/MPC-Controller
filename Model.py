import numpy as np
import math


curr_state = [0,0,0]

control_ver_0 = [1,0]
control_ver_1 = [1,0]
control_ver_2 = [1,0]
control_ver_3 = [1,0]
control_ver_4 = [1,0]

prediacted_state =[]
control_matrix = [ control_ver_0, control_ver_1, control_ver_2, control_ver_3, control_ver_4]



# predicted stste for dt = 0.1 seconds
def predict_state_1(curr_state, control_ver_0, dt):
	global prediacted_state

	x0 = curr_state[0]
	y0 = curr_state[1]
	r0 = curr_state[2]
	v0 = control_ver_0[0]
	w0 = control_ver_0[1]
	dt = dt

	A = np.array([ [1,0,0], [0,1,0], [0,0,1]])
	B = np.array([ [ math.cos(r0)*dt, 0], [math.sin(r0)*dt, 0], [0, dt]])

	predict_state_1 = np.dot(A, curr_state) + np.dot(B, control_ver_0)
	print("predict_state_1 : ", predict_state_1)
	prediacted_state.append(predict_state_1)

	return predict_state_1


def predict_state_2(curr_state, control_ver_0, control_ver_1, dt):
	global prediacted_state

	state_1 = predict_state_1(curr_state, control_ver_0, dt)
	x1 = state_1[0]
	y1 = state_1[1]
	r1 = state_1[2]
	v1 = control_ver_1[0]
	w1 = control_ver_1[1]
	
	A = np.array([ [1,0,0], [0,1,0], [0,0,1]])
	B = np.array([ [ math.cos(r1)*dt, 0], [math.sin(r1)*dt, 0], [0, dt]])
	
	predict_state_2 = np.dot(A, state_1) + np.dot(B, control_ver_1)
	print("predict_state_2 : ", predict_state_2)
	prediacted_state.append(predict_state_2)
	
	return predict_state_2


def predict_state_3(curr_state, control_ver_0, control_ver_1,control_ver_2, dt):
	global prediacted_state
	
	state_2 = predict_state_2(curr_state, control_ver_0, control_ver_1, dt)
	x2 = state_2[0]
	y2 = state_2[1]
	r2 = state_2[2]
	v2 = control_ver_2[0]
	w2 = control_ver_2[1]
	
	A = np.array([ [1,0,0], [0,1,0], [0,0,1]])
	B = np.array([ [ math.cos(r2)*dt, 0], [math.sin(r2)*dt, 0], [0, dt]])
	
	predict_state_3 = np.dot(A, state_2) + np.dot(B, control_ver_2)
	print("predict_state_3 : ", predict_state_3)
	prediacted_state.append(predict_state_3)
	
	return predict_state_3


def predict_state_4(curr_state, control_ver_0, control_ver_1, control_ver_2, control_ver_3, dt):
	global prediacted_state
	
	state_3 = predict_state_3(curr_state, control_ver_0, control_ver_1, control_ver_2, dt)
	x3 = state_3[0]
	y3 = state_3[1]
	r3 = state_3[2]
	v3 = control_ver_3[0]
	w3 = control_ver_3[1]
	
	A = np.array([ [1,0,0], [0,1,0], [0,0,1]])
	B = np.array([ [ math.cos(r3)*dt, 0], [math.sin(r3)*dt, 0], [0, dt]])
	
	predict_state_4 = np.dot(A, state_3) + np.dot(B, control_ver_3)
	print("predict_state_4 : ", predict_state_4)
	prediacted_state.append(predict_state_4)
	
	return predict_state_4


def predict_state_5(curr_state, control_ver_0, control_ver_1, control_ver_2, control_ver_3, control_ver_4, dt):
	global prediacted_state
	
	state_4 = predict_state_4(curr_state, control_ver_0, control_ver_1, control_ver_2, control_ver_3, dt)
	x4 = state_4[0]
	y4 = state_4[1]
	r4 = state_4[2]
	v4 = control_ver_4[0]
	w4 = control_ver_4[1]
	
	A = np.array([ [1,0,0], [0,1,0], [0,0,1]])
	B = np.array([ [ math.cos(r4)*dt, 0], [math.sin(r4)*dt, 0], [0, dt]])
	
	predict_state_5 = np.dot(A, state_4) + np.dot(B, control_ver_4)
	print("predict_state_5 : ", predict_state_5)
	prediacted_state.append(predict_state_5)
	
	return predict_state_5


def cost_FN( control_matrix):
	curr_state = [0,0,0]
	follow_path = [ [2, 0, 0], [4, 0, 0], [6, 0, 0], [8, 0, 0], [10, 0, 0]]
	weight = [ 1, 1, 1, 1, 1]
	predict_state_5(curr_state, control_matrix[0], control_matrix[1], control_matrix[2], control_matrix[3], control_matrix[4], 1)

	J = weight[0]*(follow_path[0] - prediacted_state[0])**2 + weight[1]*(follow_path[1] - prediacted_state[1])**2 + weight[2]*(follow_path[2] - prediacted_state[2])**2 + weight[3]*(follow_path[3] - prediacted_state[3])**2 + weight[4]*(follow_path[4] - prediacted_state[4])**2
	return J


print( cost_FN( control_matrix))

# predict_state_3(curr_state, control_ver_0, control_ver_1, control_ver_2, 0.1)