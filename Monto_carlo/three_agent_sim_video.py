import numpy as np
from math import pi,cos,sin,atan2,sqrt
from time import sleep
import matplotlib.pyplot as plt 
import random
import copy
import random

initial_position = np.array([[1,0,pi/2],[0,0,pi/2]])						# initail position of A and B respectively
 
#target = np.array([[18,84],[30,84],[30,23.5]])								# target for A nd B respectively

target = np.array([[50,55,0],[25,55,0]])
#random.randrange(20, 50, 3)
matrix_pos = np.around(np.array([15,15]), decimals = 3)

robot_A_pos = np.around(np.array([0,0,pi/2]), decimals = 3)

robot_B_pos = np.around(np.array([10,0,pi/2]), decimals = 3)

robot_C_pos = np.around(np.array([20,0,pi/2]), decimals = 3)

robot_D_pos = np.around(np.array([20,10,pi/2]), decimals = 3)

#robot_E_pos = np.around(np.array([10,10,pi/2]), decimals = 3)

bot_conf = np.array([['_','_','_'],
					 ['_','_','_'],
					 ['A','B','C']])
bot_list = ['A','B','C']
n = len(bot_list)	
number_of_targets = len(target)				 
dt = 0.1
vel_err_prev_X = 0
vel_err_prev_Y = 0
ang_err_prev = 0
prev_err_A = [0,0]
prev_err_B = [0,0]
prev_err_C = [0,0]
prev_err_D = [0,0]
prev_err_E = [0,0]
targ_rwrd = 5
matrix_target_list = np.array([[15,15],[15,75],[45,75],[45,15],[75,15],[75,75]])

#MATRIX NAVIGATION

def error_calculation(matrix_pos,next_target):									## error calculation for matrix navigation
	global vel_err_X,vel_err_Y,targ
	#targ = np.array([np.median(next_target[:,0]),np.median(next_target[:,1])])
	targ = [next_target[0],next_target[1]]
	vel_err_X = -np.around((matrix_pos[0] - targ[0]),decimals = 2)					##distance between matrix and target for driving the velocity
	vel_err_Y = -np.around((matrix_pos[1] - targ[1]),decimals = 2)
	#ang_err = np.around(matrix_pos[2] - atan2((-matrix_pos[1] + targ[1]),(-matrix_pos[0] + targ[0])),decimals = 2)
	return vel_err_X,vel_err_Y,targ

def error_calculation_targ(matrix_pos,next_target):									## error calculation for matrix navigation
	global vel_err_X,vel_err_Y,targ_new
	# if len(next_target.shape) == 1:
	# 	targ_new = [np.array([matrix_pos[0],np.median(next_target[:][1])])]
	# 	#vel_err = np.around(sqrt((matrix_pos[0] - targ_new[0][0])**2 + (matrix_pos[1] - targ_new[0][1])**2),decimals = 2)					##distance between matrix and target for driving the velocity
	# 	vel_err_Y = np.around(sqrt((matrix_pos[1] - targ_new[0][1])**2),decimals = 2)					##distance between matrix and target for driving the velocity
		
		
	# 	#ang_err = np.around(matrix_pos[2] - atan2((-matrix_pos[1] + targ_new[0][1]),(-matrix_pos[0] + targ_new[0][0])),decimals = 2)
	# 	vel_err_X = 0
	# 	targ_new = targ_new[0]
	# else:
	targ_new = np.array([matrix_pos[0],np.median(next_target[:,1])])	
	#targ = [next_target[0],next_target[1]]
		#vel_err_X = np.around((matrix_pos[0] - targ_new[0]),decimals = 2)					##distance between matrix and target for driving the velocity
	vel_err_X = 0
	vel_err_Y = -np.around((matrix_pos[1] - targ_new[1]),decimals = 2)	
		
		#ang_err = np.around(matrix_pos[2] - atan2((-matrix_pos[1] + targ_new[1]),(-matrix_pos[0] + targ_new[0])),decimals = 2)
	return vel_err_X,vel_err_Y,targ_new

def matrix_control_variable(vel_err_X,vel_err_Y):						#control variable generation for the matrix
	global u
	#v = 0
	#if abs(ang_err) < 0.4:
	#v_Y = -0.1*vel_err_Y + 0.15*(vel_err_Y - vel_err_prev_Y)
	#v_X = -0.1*vel_err_X + 0.15*(vel_err_X - vel_err_prev_X)
	#omega = -2*ang_err #- 0.2*(ang_err - ang_err_prev)
	#print(vel_err_X,vel_err_Y)
	if abs(vel_err_Y) <= 1:
		v_Y = 1*vel_err_Y			
	elif (vel_err_Y) > 1:
		v_Y = 8
	elif (vel_err_Y) < -1:
		v_Y = -8	
		
	if abs(vel_err_X) <= 1:
		v_X = 1*vel_err_X			
	elif (vel_err_X) > 1:
		v_X = 8
	elif (vel_err_X) < -1:
		v_X = -8	
			
	# vel_err_prev_X = vel_err_X
	# vel_err_prev_Y = vel_err_Y
	#ang_err_prev = ang_err
	u = np.array([v_X,v_Y])
	return u

def update_matrix_pose(u):										  # matrix pose being updated according to the control variable generated
	global matrix_pos
	#si = matrix_pos[2] + u[1]*dt
	x = matrix_pos[0] + u[0]*dt
	y = matrix_pos[1] + u[1]*dt
	matrix_pos = np.array([x,y])
	return matrix_pos
	
def initialize_local_wrt_global(matrix_pos):						## the position according to the global frame is assigned to the local coordinates
    d = 5 	# distance between two adgecent nodes
    global mat_wrt_global
    x,y = matrix_pos
    si = pi/2
	
    x0,y0 = np.array([x,y]) + 2*np.array([d*cos(si),d*sin(si)])
    x1,y1 = np.array([x,y]) + np.array([d*cos(si),d*sin(si)])
    x2,y2 = np.array([x,y]) 
    x3,y3 = np.array([x,y]) - np.array([d*cos(si),d*sin(si)])
    x4,y4 = np.array([x,y]) - 2*np.array([d*cos(si),d*sin(si)])
    mat_wrt_global  = np.around(np.array([  [ np.array([x1,y1]) - np.array([d*sin(si),-d*cos(si)]) , np.array([x1,y1]) , np.array([x1,y1]) + np.array([d*sin(si),-d*cos(si)])],
								 			[ np.array([x2,y2]) - np.array([d*sin(si),-d*cos(si)]) , np.array([x2,y2]) , np.array([x2,y2]) + np.array([d*sin(si),-d*cos(si)])],
										    [ np.array([x3,y3]) - np.array([d*sin(si),-d*cos(si)]) , np.array([x3,y3]) , np.array([x3,y3]) + np.array([d*sin(si),-d*cos(si)])]]), decimals = 3)
	
    
    # mat_wrt_global_A = copy.deepcopy(mat_wrt_global)
    # mat_wrt_global_B = copy.deepcopy(mat_wrt_global)
    # mat_wrt_global_C = copy.deepcopy(mat_wrt_global)
    # if kk == 1:
	#     if 0 in P:
	#     	for i in range(len(beta)):
	#     		if beta[i][2] == 0.0:		
	#     			if len(target_loc.shape) == 1:
	#     				mat_wrt_global_A[beta[i][0]][beta[i][1]] = target_loc[0:2]
	#     			else:
	#     				mat_wrt_global_A[beta[i][0]][beta[i][1]] = [target_loc[ np.where(target_capture == 0.0)[0][0] ][ 0 ], target_loc[ np.where(target_capture == 0.0)[0][0] ][ 1 ]] 			 

	#     if 1 in P:		
	#     	for i in range(len(beta)):
	#     		if beta[i][2] == 1.0:
	#     			if len(target_loc.shape) == 1:
	#     				mat_wrt_global_B[beta[i][0]][beta[i][1]] = target_loc[0:2]
	#     			else:
	#     				mat_wrt_global_B[beta[i][0]][beta[i][1]] = [target_loc[ np.where(target_capture == 1.0)[0][0] ][ 0 ], target_loc[ np.where(target_capture == 1.0)[0][0] ][ 1 ]] 			 
	#     if 2 in P:		
	#     	for i in range(len(beta)):
	#     		if beta[i][2] == 2.0:
	#     			if len(target_loc.shape) == 1:
	#     				mat_wrt_global_C[beta[i][0]][beta[i][1]] = target_loc[0:2]
	#     			else:
	#     				mat_wrt_global_C[beta[i][0]][beta[i][1]] = [target_loc[ np.where(target_capture == 2.0)[0][0] ][ 0 ], target_loc[ np.where(target_capture == 2.0)[0][0] ][ 1 ]] 			 
    # return mat_wrt_global,mat_wrt_global_A,mat_wrt_global_B,mat_wrt_global_C
    
    return mat_wrt_global


def final_local_wrt_global(target_n,final_heading_angle):						## the position according to the global frame is assigned to the local coordinates
	d = 5 	# distance between two adgecent nodes
	#global final_mat_wrt_global
	x,y = target_n
	#si = final_heading_angle 
	si = pi/2
	
	x0,y0 = np.array([x,y]) + 2*np.array([d*cos(si),d*sin(si)])
	x1,y1 = np.array([x,y]) + np.array([d*cos(si),d*sin(si)])
	x2,y2 = np.array([x,y]) 
	x3,y3 = np.array([x,y]) - np.array([d*cos(si),d*sin(si)])
	x4,y4 = np.array([x,y]) - 2*np.array([d*cos(si),d*sin(si)])
	final_mat_wrt_global  = np.around(np.array([  [ np.array([x1,y1]) - np.array([d*sin(si),-d*cos(si)]) , np.array([x1,y1]) , np.array([x1,y1]) + np.array([d*sin(si),-d*cos(si)])],
								 				  [ np.array([x2,y2]) - np.array([d*sin(si),-d*cos(si)]) , np.array([x2,y2]) , np.array([x2,y2]) + np.array([d*sin(si),-d*cos(si)])],
												  [ np.array([x3,y3]) - np.array([d*sin(si),-d*cos(si)]) , np.array([x3,y3]) , np.array([x3,y3]) + np.array([d*sin(si),-d*cos(si)])]]), decimals = 3)	
	return final_mat_wrt_global			
	
def check_if_reached(matrix_pos,tar):										##This funtion is finally aimed towards deciding the final position of the matrix required and if reached or not
	global ckr
	dist = sqrt((matrix_pos[0] - tar[0])**2 + (matrix_pos[1] - tar[1])**2)
	if dist <= 0.9:
		ckr = 1
		#print("hello")
	else:
		ckr = 0	
	return ckr

	
###############################################	
def robot_pos_wrt_local():
	#global pos
	p = []
	Bot = []		
	for i in range(n):
		Bot.append(	np.where( bot_conf == bot_list[i] ) )
		p.append( [ Bot[i][0][0], Bot[i][1][0] ] )
	return p

## Navigation of robot A		
def error_calculation_ROBOT_A(pos,mat_wrt_global):
	global err_A
	targ_a = mat_wrt_global[pos[0][0]][pos[0][1]]
	vel_err_A = np.around(sqrt((robot_A_pos[0] - targ_a[0])**2 + (robot_A_pos[1] - targ_a[1])**2),decimals = 2)				##distance between matrix and target for driving the velocity
	ang_err_A = np.around((robot_A_pos[2] - atan2((-robot_A_pos[1] + targ_a[1]),(-robot_A_pos[0] + targ_a[0]))),decimals = 2)
	err_A = np.array([vel_err_A,ang_err_A])
	return err_A

def control_variable_ROBOT_A(err_A):
	global prev_err_A,u_A
	omega = -10*err_A[1] #- 0.2*(ang_err - ang_err_prev)
	if err_A[0] <= 0.05:
		if err_A[1] < pi/3:
			v = 2*err_A[0] #+ 0.15*(err_A[0] - prev_err_A[0])
			if v > 5:
				v = 5
		else:
			v = 0
	else:
		if abs(err_A[1]) < 0.1:
			#v = 1*err_A[0] #+ 0.15*(err_A[0] - prev_err_A[0])
			v = 20
			omega = 0
		else:
			v = 0	
	
	
	# if omega > 2:
	# 	omega = 2
	# elif omega < -2:
	# 	omega = -2
	u_A = np.array([v,omega])
	return u_A

def robot_A_navigation(u_A):
	global robot_A_pos
	si = robot_A_pos[2] + u_A[1]*dt
	x = robot_A_pos[0] + u_A[0]*cos(si)*dt
	y = robot_A_pos[1] + u_A[0]*sin(si)*dt
	robot_A_pos = np.array([x,y,si])
	return robot_A_pos

## Navigation of robot E		
def error_calculation_ROBOT_E(pos,mat_wrt_global):
	global err_E
	targ_e = mat_wrt_global[pos[4][0]][pos[4][1]]
	vel_err_E = np.around(sqrt((robot_E_pos[0] - targ_e[0])**2 + (robot_E_pos[1] - targ_e[1])**2),decimals = 2)				##distance between matrix and target for driving the velocity
	ang_err_E = np.around((robot_E_pos[2] - atan2((-robot_E_pos[1] + targ_e[1]),(-robot_E_pos[0] + targ_e[0]))),decimals = 2)
	err_E = np.array([vel_err_E,ang_err_E])
	return err_E

def control_variable_ROBOT_E(err_E):
	global prev_err_E,u_E
	omega = -10*err_E[1] #- 0.2*(ang_err - ang_err_prev)
	if err_E[0] <= 0.05:
		if err_E[1] < pi/3:
			v = 2*err_E[0] #+ 0.15*(err_A[0] - prev_err_A[0])
			if v > 5:
				v = 5
		else:
			v = 0
	else:
		if abs(err_E[1]) < 0.1:
			#v = 1*err_A[0] #+ 0.15*(err_A[0] - prev_err_A[0])
			v = 20
			omega = 0
		else:
			v = 0	

	
	
	# if omega > 2:
	# 	omega = 2
	# elif omega < -2:
	# 	omega = -2
	u_E = np.array([v,omega])
	return u_E

def robot_E_navigation(u_E):
	global robot_E_pos
	si = robot_E_pos[2] + u_E[1]*dt
	x = robot_E_pos[0] + u_E[0]*cos(si)*dt
	y = robot_E_pos[1] + u_E[0]*sin(si)*dt
	robot_E_pos = np.array([x,y,si])
	return robot_E_pos

## Navigation of robot D

def error_calculation_ROBOT_D(pos,mat_wrt_global):
	global err_D
	targ_d = mat_wrt_global[pos[3][0]][pos[3][1]]
	vel_err_D = np.around(sqrt((robot_D_pos[0] - targ_d[0])**2 + (robot_D_pos[1] - targ_d[1])**2),decimals = 2)				##distance between matrix and target for driving the velocity
	ang_err_D = np.around((robot_D_pos[2] - atan2((-robot_D_pos[1] + targ_d[1]),(-robot_D_pos[0] + targ_d[0]))),decimals = 2)
	err_D = np.array([vel_err_D,ang_err_D])
	return err_D

def control_variable_ROBOT_D(err_D):
	global prev_err_D,u_D
	omega = -10*err_D[1] #- 0.2*(ang_err - ang_err_prev)
	if err_D[0] <= 0.05:
		if err_D[1] < pi/3:
			v = 2*err_D[0] #+ 0.15*(err_A[0] - prev_err_A[0])
			if v > 5:
				v = 5
		else:
			v = 0
	else:
		if abs(err_D[1]) < 0.1:
			#v = 1*err_A[0] #+ 0.15*(err_A[0] - prev_err_A[0])
			v = 20
			omega = 0
		else:
			v = 0	

	
	
	# if omega > 2:
	# 	omega = 2
	# elif omega < -2:
	# 	omega = -2
	u_D = np.array([v,omega])
	return u_D

def robot_D_navigation(u_D):
	global robot_D_pos
	si = robot_D_pos[2] + u_D[1]*dt
	x = robot_D_pos[0] + u_D[0]*cos(si)*dt
	y = robot_D_pos[1] + u_D[0]*sin(si)*dt
	robot_D_pos = np.array([x,y,si])
	return robot_D_pos
		
## Navigation of robot C 
def error_calculation_ROBOT_C(pos,mat_wrt_global):
	global err_C
	targ_c = mat_wrt_global[pos[2][0]][pos[2][1]]
	vel_err_C = np.around(sqrt((robot_C_pos[0] - targ_c[0])**2 + (robot_C_pos[1] - targ_c[1])**2),decimals = 2)					##distance between matrix and target for driving the velocity
	ang_err_C = np.around((robot_C_pos[2] - atan2((-robot_C_pos[1] + targ_c[1]),(-robot_C_pos[0] + targ_c[0]))),decimals = 2)
	err_C = np.array([vel_err_C,ang_err_C])
	return err_C

def control_variable_ROBOT_C(err_C):
	global prev_err_C,u_C
	omega = -10*err_C[1] #- 0.2*(ang_err - ang_err_prev)
	if err_C[0] <= 0.05:
		if err_C[1] < pi/3:
			v = 2*err_C[0] #+ 0.15*(err_A[0] - prev_err_A[0])
			if v > 5:
				v = 5
		else:
			v = 0
	else:
		if abs(err_C[1]) < 0.1:
			#v = 1*err_A[0] #+ 0.15*(err_A[0] - prev_err_A[0])
			v = 20
			omega = 0
		else:
			v = 0	
	
	# if omega > 2:
	# 	omega = 2
	# elif omega < -2:
	# 	omega = -2
	u_C = np.array([v,omega])
	return u_C

def robot_C_navigation(u_C):
	global robot_C_pos
	si = robot_C_pos[2] + u_C[1]*dt
	x = robot_C_pos[0] + u_C[0]*cos(si)*dt
	y = robot_C_pos[1] + u_C[0]*sin(si)*dt
	robot_C_pos = np.array([x,y,si])
	return robot_C_pos	
				
## Navigation of robot B				
def error_calculation_ROBOT_B(pos,mat_wrt_global):
	global err_B
	targ_b = mat_wrt_global[pos[1][0]][pos[1][1]]
	vel_err_B = np.around(sqrt((robot_B_pos[0] - targ_b[0])**2 + (robot_B_pos[1] - targ_b[1])**2),decimals = 2)					##distance between matrix and target for driving the velocity
	ang_err_B = np.around((robot_B_pos[2] - atan2((-robot_B_pos[1] + targ_b[1]),(-robot_B_pos[0] + targ_b[0]))),decimals = 2)
	err_B = np.array([vel_err_B,ang_err_B])
	return err_B	
	
def control_variable_ROBOT_B(err_B):
	global prev_err_B,u_B	
	omega = -10*err_B[1] #- 0.2*(ang_err - ang_err_prev)
	prev_err_B = err_B
	if err_B[0] <= 0.05:
		if err_B[1] < pi/3:
			v = 2*err_B[0] + 0.15*(err_B[0] - prev_err_B[0])
			if v > 5:
				v = 5
		else:
			v = 0
	else:
		if abs(err_B[1]) < 0.1:
			#v = 1*err_B[0] + 0.15*(err_B[0] - prev_err_B[0])
			v = 20
			omega = 0
		else:
			v = 0
							
	# if omega > 2:
	# 	omega = 2
	# elif omega < -2:
	# 	omega = -2		
	u_B = np.array([v,omega])		
	return u_B
	
def robot_B_navigation(u_B):
	global robot_B_pos
	si = robot_B_pos[2] + u_B[1]*dt
	x = robot_B_pos[0] + u_B[0]*cos(si)*dt
	y = robot_B_pos[1] + u_B[0]*sin(si)*dt
	robot_B_pos = np.array([x,y,si])	
	return robot_B_pos
#######################################

def local_matrix_target_for_reconf(target_location,x_mat_wrt_global,alpha):
	global R,Q,P,R_dash       	
	Q = []
	R = []
	P = []	
	task_distance = []
	robot_pose = [robot_A_pos,robot_B_pos,robot_C_pos] ## Change here
	# T = []
	# T.append(alpha)
	T = alpha
	T_dash = copy.deepcopy(T)
	z = target_location.shape
	# #T = [[0,0],[0,0],[0,0]]
	# if len(target_location.shape) == 1:
	# 	#for i in range(z[0]):
	# 	P.append(target_location[2])
	# 	for p in range(5):
	# 		for q in range(5):			
	# 			dist1 = sqrt((x_mat_wrt_global[p][q][0] - target_location[0])**2 + (x_mat_wrt_global[p][q][1] - target_location[1])**2)				
	# 			Q.append(dist1)
	# 		#for j in range (n):
						
						
	#else:				
	for i in range(z[0]):
		#P.append(target_location[i][2])
		for p in range(3):
			for q in range(3):			
				dist1 = sqrt((x_mat_wrt_global[p][q][0] - target_location[i][0])**2 + (x_mat_wrt_global[p][q][1] - target_location[i][1])**2)				
				Q.append(dist1)
				
				#print(Q)
	for j in range (n):
		for i in range(z[0]):
			dist2 = sqrt((robot_pose[j][0] - target_location[i][0])**2 + (robot_pose[j][1] - target_location[i][1])**2)
			R.append(dist2)
			
	if len(Q) > 0:
		Q = np.around(Q,decimals = 2)	
		Q = Q.reshape(z[0],9)
		
	if len(R) > 0:
		R = np.around(R,decimals = 2)	
		R = R.reshape(n,z[0])
	
	R_dash = copy.deepcopy(R)
	for i in range (z[0]):
		
		a1 = R_dash[:,i]
		taskI_to_robot = np.where(a1 == np.amin(a1))[0][0]
		#print("Task I to robot",taskI_to_robot,R_dash)
		R_dash[taskI_to_robot,:] = 1000
		P.append(taskI_to_robot)
			
			
	if 0 in P:
		x = [0,1,2,3,4,5,6,7,8,9]
		q1 = Q[:][P.index(0)].reshape(3,3)
		T1 = np.where(q1 == np.amin(q1))
		T[0][0] = T1[0][0]
		T[0][1] = T1[1][0]
		for k in range(n):
			if k == 0:
				continue
			else:
				if 5*T[0][0] + T[0][1] == 5*T[k][0] + T[k][1]:
					for l in range(n):
						if l == k:
							continue
						else:
							if 5*T[l][0] + T[l][1] in x:
								x.remove(5*T[l][0] + T[l][1])
					y = random.choice(x)
					T[k][0] = int(y/5)
					T[k][1] = int(y%5)
					
	if 1 in P:
		x = [0,1,2,3,4,5,6,7,8,9]
		q2 = Q[:][P.index(1)].reshape(3,3)
		T2 = np.where(q2 == np.amin(q2))
		T[1][0] = T2[0][0]
		T[1][1] = T2[1][0]	
		for k in range(n):
			if k == 1:
				continue
			else:
				if 5*T[1][0] + T[1][1] == 5*T[k][0] + T[k][1]:
					for l in range(n):
						if l == k:
							continue
						else:	
							if 5*T[l][0] + T[l][1] in x:
								x.remove(5*T[l][0] + T[l][1])
					y = random.choice(x)
					T[k][0] = int(y/5)
					T[k][1] = int(y%5)
	if 2 in P:
		x = [0,1,2,3,4,5,6,7,8,9]
		q3 = Q[:][P.index(2)].reshape(3,3)	
		T3 = np.where(q3 == np.amin(q3))
		T[2][0] = T3[0][0]
		T[2][1] = T3[1][0]
		for k in range(n):
			if k == 2:
				continue
			else:
				if 5*T[2][0] + T[2][1] == 5*T[k][0] + T[k][1]:
					for l in range(n):
						if l == k:
							continue
						else:	
							if 5*T[l][0] + T[l][1] in x:
								x.remove(5*T[l][0] + T[l][1])
					y = random.choice(x)
					T[k][0] = int(y/5)
					T[k][1] = int(y%5)
	
	if 3.0 in P:
		x = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24]
		q4 = Q[:][P.index(3.0)].reshape(5,5)
		T4 = np.where(q4 == np.amin(q4))
		T[3][0] = T4[0][0]
		T[3][1] = T4[1][0]
		for k in range(n):
			if k == 3:
				continue
			else:
				if 5*T[3][0] + T[3][1] == 5*T[k][0] + T[k][1]:
					for l in range(n):
						if l == k:
							continue
						else:
							if 5*T[l][0] + T[l][1] in x:
								x.remove(5*T[l][0] + T[l][1])
					y = random.choice(x)
					T[k][0] = int(y/5)
					T[k][1] = int(y%5)			
		
	if 4.0 in P:
		x = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24]
		q5 = Q[:][P.index(4.0)].reshape(5,5)
		T5 = np.where(q5 == np.amin(q5))
		T[4][0] = T5[0][0]
		T[4][1] = T5[1][0]
		for k in range(n):
			if k == 4:
				continue
			else:
				if 5*T[4][0] + T[4][1] == 5*T[k][0] + T[k][1]:
					for l in range(n):
						if l == k:
							continue
						else:
							if 5*T[l][0] + T[l][1] in x:
								x.remove(5*T[l][0] + T[l][1])
					y = random.choice(x)
					T[k][0] = int(y/5)
					T[k][1] = int(y%5)


	return T

def payoff_compute(T,pose):       ## Calculate the pay offs for each robot	
	
	pof = []
	c1 = c2 = c3 =c4 = c5= 0	
	for i in range(n):		
		est_pose = [[pose[i][0]+1, pose[i][1]],						# front     dec = 0
					[pose[i][0]-1, pose[i][1]],						# back			  1
					[pose[i][0], pose[i][1]-1],						# right           2
					[pose[i][0], pose[i][1]+1],						# left			  3
					[pose[i][0], pose[i][1]]]							# STAY            4
		
		for j in range (n):
			if j == i:
				continue
			else:
				if (est_pose[0] == pose[j]):
					c1 = 3
				if (est_pose[1] == pose[j]):
					c2 = 3
				if (est_pose[2] == pose[j]):
					c3 = 3
				if (est_pose[3] == pose[j]):
					c4 = 3
				if (est_pose[4] == pose[j]):
					c5 = 3

		k1 = (abs(est_pose[0][0] - pose[i][0]) + abs(est_pose[0][1] - pose[i][1]))**2		
		k2 = (abs(est_pose[1][0] - pose[i][0]) + abs(est_pose[1][1] - pose[i][1]))**2
		k3 = (abs(est_pose[2][0] - pose[i][0]) + abs(est_pose[2][1] - pose[i][1]))**2
		k4 = (abs(est_pose[3][0] - pose[i][0]) + abs(est_pose[3][1] - pose[i][1]))**2
		k5 = (abs(est_pose[4][0] - pose[i][0]) + abs(est_pose[4][1] - pose[i][1]))**2
						
		pof.append( [ targ_rwrd -  3*abs(est_pose[0][0] - T[i][0]) - 3*abs(est_pose[0][1] - T[i][1]) - k1 - c1,
	   			   	  targ_rwrd -  3*abs(est_pose[1][0] - T[i][0]) - 3*abs(est_pose[1][1] - T[i][1]) - k2 - c2,
	   			      targ_rwrd -  3*abs(est_pose[2][0] - T[i][0]) - 3*abs(est_pose[2][1] - T[i][1]) - k3 - c3,
	   			      targ_rwrd -  3*abs(est_pose[3][0] - T[i][0]) - 3*abs(est_pose[3][1] - T[i][1]) - k4 - c4,
					  targ_rwrd -  3*abs(est_pose[4][0] - T[i][0]) - 3*abs(est_pose[4][1] - T[i][1])] )	
	return pof
	
def decision(pof,posin):				## deciding the next step of each robot
	x = []
	p = []
	est_pose = []
	dec= []

	action = []
	estimate_pos = copy.deepcopy(pose)

	pof_dash = copy.deepcopy(pof)
	
	for i in range (n):
		a = pof_dash[i].index(max(pof_dash[i])) 
		#a =  np.where(pof_dash[i] == np.amax(pof_dash[i]))
		#print(i,":i .... a:",a)
		pof_dash[i][a] = -10
		if a == 0:
			if estimate_pos[i][0]+1 >= 0 and estimate_pos[i][0]+1 < 3:
				estimate_pos[i] = [estimate_pos[i][0]+1, estimate_pos[i][1]]
		elif a == 1:
			if estimate_pos[i][0]-1 >= 0 and estimate_pos[i][0]-1 < 3:
				estimate_pos[i] = [estimate_pos[i][0]-1, estimate_pos[i][1]]
		elif a == 2:
			if estimate_pos[i][1]-1 >= 0 and estimate_pos[i][1]-1 < 3:
				estimate_pos[i] = [estimate_pos[i][0], estimate_pos[i][1]-1]
		elif a == 3:
			if estimate_pos[i][1]+1 >= 0 and estimate_pos[i][1]+1 < 3:
				estimate_pos[i] = [estimate_pos[i][0], estimate_pos[i][1]+1]
		action.append(a)
		#print(action, "---------", i)

	for i in range (n-1):
		#for j in range():
		for j in range (i+1):
			if estimate_pos[i+1] == estimate_pos[j]:
				a = pof_dash[i+1].index(max(pof_dash[i+1]))
				#a = np.amax(pof_dash[i+1])
				pof_dash[i+1][a] = -10
				if a == 0:
					if estimate_pos[i+1][0]+1 >= 0 and estimate_pos[i+1][0]+1 < 3:
						estimate_pos[i+1] = [estimate_pos[i+1][0]+1, estimate_pos[i+1][1]]
				elif a == 1:
					if estimate_pos[i+1][0]-1 >= 0 and estimate_pos[i+1][0]-1 < 3:
						estimate_pos[i+1] = [estimate_pos[i+1][0]-1, estimate_pos[i+1][1]]
				elif a == 2:
					if estimate_pos[i+1][1]-1 >= 0 and estimate_pos[i+1][1]-1 < 3:
						estimate_pos[i+1] = [estimate_pos[i+1][0], estimate_pos[i+1][1]-1]
				elif a == 3:
					if estimate_pos[i+1][1]+1 >= 0 and estimate_pos[i+1][1]+1 < 3:
						estimate_pos[i+1] = [estimate_pos[i+1][0], estimate_pos[i+1][1]+1]
				action[i+1] = a


# ######################
# 	for i in range (n):
# 		#print(i,"i")
# 	#	x = pof[i].index(max(pof[i]))
# 		est_pose = []
# 		p = pof[i]
# 		m = max(pof[i])
# 		x = [k for k, j in enumerate(pof[i]) if j == m]
		
# 		#for i in range (n):			
# 		if 0 in x:
# 			if posin[i][0]+1 < 0 or posin[i][0]+1 > 4:
# 				x.remove(0)
# 			else:		
# 				est_pose.append((posin[i][0]+1, posin[i][1]))			
# 				#pos[i] = [pos[i][0]+1, pos[i][1]]
# 		if 1 in x:
# 			if posin[i][0]-1 < 0 or posin[i][0]-1 > 4:
# 				x.remove(1)
# 			else:				
# 				est_pose.append((posin[i][0]-1, posin[i][1]))
# 				#pos[i] = [pos[i][0]-1, pos[i][1]]
# 		if 2 in x:
# 			if posin[i][1]-1 < 0 or posin[i][1]-1 > 4:
# 				x.remove(2)
# 			else:	
# 				est_pose.append((posin[i][0], posin[i][1]-1))
# 				#pos[i] = [pos[i][0], pos[i][1]-1]
# 		if 3 in x:
# 			if posin[i][1]+1 < 0 or posin[i][1]+1 > 4:
# 				x.remove(3)
# 			else:
# 				est_pose.append((posin[i][0], posin[i][1]+1))
# 				#pos[i] = [pos[i][0], pos[i][1]+1]
		
# 		for j in range (n):
# 			#print(j,"j")
# 			est_pose_fr = []
# 			est = est_pose
# 			m1 = max(pof[j])
# 			x1 = [p for p, l in enumerate(pof[j]) if l == m1]
			
# 			if j == i:
# 				#print("hello")
# 				x = [random.choice(x)]
# 				#print(x)
# 				break
# 			else:
				
# 				if dec[j] == [0]:							
# 					est_pose_fr.append((posin[j][0]+1, posin[j][1]))
# 					#print(est_pose_fr)			
# 						#pos[i] = [pos[i][0]+1, pos[i][1]]
# 				if dec[j] == [1]:
# 					est_pose_fr.append((posin[j][0]-1, posin[j][1]))
# 						#pos[j] = [pos[j][0]-1, pos[j][1]]
# 				if dec[j] == [2]:
# 					est_pose_fr.append((posin[j][0], posin[j][1]-1))
# 						#pos[j] = [pos[j][0], pos[j][1]-1]
# 				if dec[j] == [3]:
# 					est_pose_fr.append((posin[j][0], posin[j][1]+1))
# 				if dec[j] == [4]:
# 					est_pose_fr.append((posin[j][0], posin[j][1]))
					
# 				if len(list(set(est) & set(est_pose_fr))) >= 1:					
# 					#x.remove(pof[j].index(max(pof[j])))	
# 					#print("hello")
# 					k = list(set(est) & set(est_pose_fr))
# 					est = list(set(est) - set(k))
									
# 					#x.remove(pof[j].index(max(pof[j])))
					
# 					if len(est) >= 1:
# 						#print("est length is 1")
# 						x = [est_pose.index(est[0])]
# 						#break
# 					else:
# 						#est = random.choice(est)
# 						#x = [est_pose.index(est)]
# 						#p[pof[j].index(max(pof[j]))]=-10
# 						p[x[0]]=-10
# 						m = max(p)
# 						x = [i for i, j in enumerate(pof[i]) if j == m]
# 						x = [random.choice(x)]
# 				else:
# 					# if len(x) == 1:
# 					# 	break		
# 					# else:	
# 					x = [random.choice(x)]
# 						#x.pop(pof[i].index(max(pof[i])))												
# 		dec.append(x)	
# 		#print(dec)	

	dec = copy.deepcopy(action)						  	
	return dec
	
def robot_pos_update_wrt_local(pose,dec):
	#global bot_conf
	# bot_conf = [['_','_','_'],
	# 			['_','_','_'],
	# 			['_','_','_']]
	for i in range (n):			
		if dec[i] == 0:
			if pose[i][0]+1 >= 0 and pose[i][0]+1 < 3:
				pose[i] = [pose[i][0]+1, pose[i][1]]
		elif dec[i] == 1:
			if pose[i][0]-1 >= 0 and pose[i][0]-1 < 3:
				pose[i] = [pose[i][0]-1, pose[i][1]]
		elif dec[i] == 2:
			if pose[i][1]-1 >= 0 and pose[i][1]-1 < 3:
				pose[i] = [pose[i][0], pose[i][1]-1]
		elif dec[i] == 3:
			if pose[i][1]+1 >= 0 and pose[i][1]+1 < 3:
				pose[i] = [pose[i][0], pose[i][1]+1]
			
		#bot_conf [pos[i][0]] [pos[i][1]] = bot_list[i]
		
	#bot_conf = np.array(bot_conf)
	return pose

def check_if_next_step_reached(pose,mat_wrt_global):					 #This function is to decide whether the required posee is reached or not
	global reconf_status
	#for robot A	
	d_A = sqrt((mat_wrt_global[pose[0][0]][pose[0][1]][0] - robot_A_pos[0])**2 + (mat_wrt_global[pose[0][0]][pose[0][1]][1] - robot_A_pos[1])**2)
	#print(d_A)
	if d_A <= 1:
		A_reached = 1
	else:
		A_reached = 0
	#for robot B
	d_B = sqrt((mat_wrt_global[pose[1][0]][pose[1][1]][0] - robot_B_pos[0])**2 + (mat_wrt_global[pose[1][0]][pose[1][1]][1] - robot_B_pos[1])**2)
	if d_B <= 1:
		B_reached = 1
	else:
		B_reached = 0
		
	d_C = sqrt((mat_wrt_global[pose[2][0]][pose[2][1]][0] - robot_C_pos[0])**2 + (mat_wrt_global[pose[2][0]][pose[2][1]][1] - robot_C_pos[1])**2)
	if d_C <= 1:
		C_reached = 1
	else:
		C_reached = 0	
	
	# d_D = sqrt((mat_wrt_global[pose[3][0]][pose[3][1]][0] - robot_D_pos[0])**2 + (mat_wrt_global[pose[3][0]][pose[3][1]][1] - robot_D_pos[1])**2)
	# if d_D <= 1:
	# 	D_reached = 1
	# else:
	# 	D_reached = 0	
		
	# d_E = sqrt((mat_wrt_global[pose[4][0]][pose[4][1]][0] - robot_E_pos[0])**2 + (mat_wrt_global[pose[4][0]][pose[4][1]][1] - robot_E_pos[1])**2)
	# if d_E <= 1:
	# 	E_reached = 1
	# else:
	# 	E_reached = 0		
		
	reconf_status = [A_reached,B_reached,C_reached]
	return reconf_status	
					
def target_check(target,matrix_pos):
	global target_loc
	target_loc = np.array([])
	#target_loc = np.append(target_loc,[ -1, -1 ,-1],axis = 0)
	kk = 0
	for t in range (number_of_targets):
		if target[t][0] < matrix_pos[0] + 15 and target[t][0] > matrix_pos[0] - 15 and target[t][1] < matrix_pos[1] + 15 and target[t][1] > matrix_pos[1] - 15:
			if target[t][2] == 0:
				if target_loc.shape[0] == 0:
					target_loc = np.array([np.append(target_loc, [ target[t][0], target[t][1] ,t],axis = 0)])
				else:
					target_loc = np.append(target_loc,[[ target[t][0], target[t][1] ,t]],axis = 0)	
				kk = 1
	
	return kk , target_loc	

def target_status(target):
	global target_new
	#target_new = copy.deepcopy(target)
	robot_pose = [robot_A_pos,robot_B_pos,robot_C_pos] ## Change here
	for i in range (number_of_targets):
		if target[i][2] == 0:
			for j in range (n):	
				#print(sqrt((robot_pose[j][0] - target[i][0])**2 + (robot_pose[j][1] - target[i][1])**2))	
				if sqrt((robot_pose[j][0] - target[i][0])**2 + (robot_pose[j][1] - target[i][1])**2) < 2:
					target[i][2] = 1
	return target		
						

if __name__ == '__main__':
	#initialize_local_wrt_global(matrix_pos)
	ckr = 0
	x1 = []
	x2 = []
	x3 = []
	x = []
	y1 = []
	y2 = []
	y3 = []
	y = []
	kk = 0
	si = None
	reconf_done  = 0
	one_time_variable = 0
	target_nullified = 0
	pose = robot_pos_wrt_local()
	t = 0
	for k in range(6):
		print("hello")
		
		initialize_local_wrt_global(matrix_pos)
		#check_if_next_step_reached(pose,mat_wrt_global)
		ckr = 0
		kk = 0
		#print(pos)
		while ckr == 0:
			prev_pos = pose
			target = target_status(target)
			kk , target_capture = target_check(target,matrix_pos)
			if kk == 1:
				error_calculation_targ(matrix_pos,target_capture)
			else:	
				error_calculation(matrix_pos,matrix_target_list[k+1])
							
			matrix_control_variable(vel_err_X,vel_err_Y)		
			update_matrix_pose(u)
			initialize_local_wrt_global(matrix_pos)
			check_if_next_step_reached(pose,mat_wrt_global)
			# if kk == 0 and target_nullified == 1:
			# 	target_nullified = 0
				
			if kk == 1 and one_time_variable == 0:
				reconf_done = 0
				error_calculation_targ(matrix_pos,target_capture)
				print(kk,"-----------------------KK----------------------")
				#si = matrix_pos[2]
				si = pi/2
				#print(pose)
				final_mat_wrt_global = final_local_wrt_global(targ_new,si)
				#print(pose)
				one_time_variable = 1
			 #print(final_local_wrt_global)
				qwerty = copy.deepcopy(pose)
				T = local_matrix_target_for_reconf(target_capture,final_mat_wrt_global,qwerty)
				#target_nullified = 0
				#print(pose)
				#print(qwerty,"----------------------")
				#kk = 0
				
			if kk == 1 and reconf_status == [1,1,1]:		#change here
				if  T == pose:	
					print("true")				
					target_nullified = 1
					one_time_variable = 0
				print (T)	
				#print (reconf_status)
				pof = payoff_compute(T,pose)
				deci = decision(pof,pose)
				#print(pose)
				pose = robot_pos_update_wrt_local(pose,deci)

				
			## ROBOT NAVIGATION		
			# robot A navigation
			if reconf_status[0] == 0:
				error_calculation_ROBOT_A(pose,mat_wrt_global)
				control_variable_ROBOT_A(err_A)
				robot_A_navigation(u_A)
			#robot B navigation
			if reconf_status[1] == 0:
				error_calculation_ROBOT_B(pose,mat_wrt_global)
				control_variable_ROBOT_B(err_B)
				robot_B_navigation(u_B)
		
			if reconf_status[2] == 0:
				error_calculation_ROBOT_C(pose,mat_wrt_global)
				control_variable_ROBOT_C(err_C)
				robot_C_navigation(u_C)	
			
			
			# if reconf_status[3] == 0:
			# 	error_calculation_ROBOT_D(pose,mat_wrt_global)
			# 	control_variable_ROBOT_D(err_D)
			# 	robot_D_navigation(u_D)		
			
			
			# if reconf_status[4] == 0:
			# 	error_calculation_ROBOT_E(pose,mat_wrt_global)
			# 	control_variable_ROBOT_E(err_E)
			# 	robot_E_navigation(u_E)			
		###########
		#print(bot_conf)
			#check_if_next_step_reached(pose,mat_wrt_global)
			check_if_reached(matrix_pos,matrix_target_list[k+1])
			t = t+dt
		###### for plotting purpose
			x.append(matrix_pos[0])
			y.append(matrix_pos[1])
			# x1.append(robot_A_pos[0])
			# y1.append(robot_A_pos[1])
			# x2.append(robot_B_pos[0])
			# y2.append(robot_B_pos[1])
			# x3.append(robot_C_pos[0])
			# y3.append(robot_C_pos[1])
			plt.plot(mat_wrt_global[0][0][0],mat_wrt_global[0][0][1],'r.')
			plt.plot(mat_wrt_global[0][1][0],mat_wrt_global[0][1][1],'r.')
			plt.plot(mat_wrt_global[0][2][0],mat_wrt_global[0][2][1],'r.')
			plt.plot(mat_wrt_global[1][0][0],mat_wrt_global[1][0][1],'r.')
			plt.plot(mat_wrt_global[1][1][0],mat_wrt_global[1][1][1],'r.')
			plt.plot(mat_wrt_global[1][2][0],mat_wrt_global[1][2][1],'r.')
			plt.plot(mat_wrt_global[2][0][0],mat_wrt_global[2][0][1],'r.')
			plt.plot(mat_wrt_global[2][1][0],mat_wrt_global[2][1][1],'r.')
			plt.plot(mat_wrt_global[2][2][0],mat_wrt_global[2][2][1],'b.')
	#plt.plot(x,y)
			plt.plot(robot_A_pos[0],robot_A_pos[1],'go')
			plt.plot(robot_B_pos[0],robot_B_pos[1],'go')
			plt.plot(robot_C_pos[0],robot_C_pos[1],'go')
			#plt.plot(robot_D_pos[0],robot_D_pos[1],'go')
			#plt.plot(robot_E_pos[0],robot_E_pos[1],'go')
			plt.plot(target[0][0],target[0][1],'gx')
			plt.plot(target[1][0],target[1][1],'gx')
			#plt.plot(target[2][0],target[2][1],'gx')
			#plt.plot(target[3][0],target[3][1],'gx')
			#plt.plot(target[4][0],target[4][1],'gx')
			# plt.plot(x1,y1)
			# plt.plot(x2,y2)
			#plt.plot(x3,y3)
			plt.axis([0,90,0,90])
			#plt.draw()
			plt.pause(0.01)
			plt.clf()
			#print(vel_err,ang_err, "errors")
			#print(u_A,"control variables of A")
			sleep(0.0100)