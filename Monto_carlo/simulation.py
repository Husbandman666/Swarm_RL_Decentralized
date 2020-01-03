import numpy as np
from math import pi,cos,sin,atan2,sqrt
from time import sleep
import matplotlib.pyplot as plt 
import random
import copy
import random

initial_position = np.array([[1,0,pi/2],[0,0,pi/2]])						# initail position of A and B respectively
 
target = np.array([[[7,78,0],[18,78,0],[22,33.5,0]],
				   [[38,30,0],[51,64,0],[45,30,0]],
				   [[78,30,0],[75,69,0],[85,77,0]]])								# target for A nd B respectively
targt_number = 3
# target = np.array([[random.randrange(0, 100, 3),random.randrange(20, 80, 3)],
# 				    [random.randrange(0, 100, 3),random.randrange(20, 80, 3)],
# 				    [random.randrange(0, 100, 3),random.randrange(20, 80, 3)],
# 				    [random.randrange(0, 100, 3),random.randrange(20, 80, 3)],
# 				    [random.randrange(0, 100, 3),random.randrange(20, 80, 3)],
# 				    [random.randrange(0, 100, 3),random.randrange(20, 80, 3)]])
#random.randrange(20, 50, 3)
matrix_pos = np.around(np.array([15,15,pi/2]), decimals = 3)

robot_A_pos = np.around(np.array([0,0,pi/2]), decimals = 3)

robot_B_pos = np.around(np.array([10,0,pi/2]), decimals = 3)

robot_C_pos = np.around(np.array([20,0,pi/2]), decimals = 3)

bot_conf = np.array([['A','B','_'],
			  		 ['_','_','_'],
					 ['_','C','_']])
bot_list = ['A','B','C']
n = len(bot_list)					 
dt = 0.1
vel_err_prev = 0
ang_err_prev = 0
prev_err_A = [0,0]
prev_err_B = [0,0]
prev_err_C = [0,0]
targ_rwrd = 5
matrix_target_list = np.array([[15,10],[15,90],[45,90],[45,10],[75,10],[75,90]])

#MATRIX NAVIGATION

def error_calculation(matrix_pos,next_target):									## error calculation for matrix navigation
	global vel_err,ang_err,targ
	#targ = np.array([np.median(next_target[:,0]),np.median(next_target[:,1])])
	targ = [next_target[0],next_target[1]]
	vel_err = np.around(sqrt((matrix_pos[0] - targ[0])**2 + (matrix_pos[1] - targ[1])**2),decimals = 2)					##distance between matrix and target for driving the velocity
	ang_err = np.around(matrix_pos[2] - atan2((-matrix_pos[1] + targ[1]),(-matrix_pos[0] + targ[0])),decimals = 2)
	return vel_err,ang_err,targ

def error_calculation_targ(matrix_pos,next_target):									## error calculation for matrix navigation
	global vel_err,ang_err,targ_new
	if len(next_target.shape) == 1:
		targ_new = [np.array([matrix_pos[0],np.median(next_target[:][1])])]
		#vel_err = np.around(sqrt((matrix_pos[0] - targ_new[0][0])**2 + (matrix_pos[1] - targ_new[0][1])**2),decimals = 2)					##distance between matrix and target for driving the velocity
		vel_err = np.around(sqrt((matrix_pos[1] - targ_new[0][1])**2),decimals = 2)					##distance between matrix and target for driving the velocity
		
		ang_err = np.around(matrix_pos[2] - atan2((-matrix_pos[1] + targ_new[0][1]),(-matrix_pos[0] + targ_new[0][0])),decimals = 2)
		targ_new = targ_new[0]
	else:
		targ_new = np.array([matrix_pos[0],np.median(next_target[:,1])])	
	#targ = [next_target[0],next_target[1]]
		vel_err = np.around(sqrt((matrix_pos[0] - targ_new[0])**2 + (matrix_pos[1] - targ_new[1])**2),decimals = 2)					##distance between matrix and target for driving the velocity
		ang_err = np.around(matrix_pos[2] - atan2((-matrix_pos[1] + targ_new[1]),(-matrix_pos[0] + targ_new[0])),decimals = 2)
	return vel_err,ang_err,targ_new

def matrix_control_variable(vel_err,ang_err):						#control variable generation for the matrix
	global u,vel_err_prev,ang_err_prev
	#v = 0
	#if abs(ang_err) < 0.4:
	v = 0.1*vel_err + 0.15*(vel_err - vel_err_prev)
	omega = -2*ang_err #- 0.2*(ang_err - ang_err_prev)
	if vel_err < 1:
		# if abs(ang_err) < 0.1:
		# 	v = 0.2*vel_err + 0.15*(vel_err - vel_err_prev)
		# else:
			v = 0
					
	else:
		if abs(ang_err) < 0.1:
			v = 10
		else:
			v = 0
	vel_err_prev = vel_err
	ang_err_prev = ang_err
	u = np.array([v,omega])
	return u

def update_matrix_pose(u):										  # matrix pose being updated according to the control variable generated
	global matrix_pos
	si = matrix_pos[2] + u[1]*dt
	x = matrix_pos[0] + u[0]*cos(si)*dt
	y = matrix_pos[1] + u[0]*sin(si)*dt
	matrix_pos = np.array([x,y,si])
	return matrix_pos
	
def initialize_local_wrt_global(matrix_pos,kk):						## the position according to the global frame is assigned to the local coordinates
	d = 5 	# distance between two adgecent nodes
	global mat_wrt_global
	x,y,si = matrix_pos
	
	x1,y1 = np.array([x,y]) + np.array([d*cos(si),d*sin(si)])
	x2,y2 = np.array([x,y]) 
	x3,y3 = np.array([x,y]) - np.array([d*cos(si),d*sin(si)])

	mat_wrt_global  = np.around(np.array([  [  np.array([x1,y1]) - np.array([d*sin(si),-d*cos(si)]) , np.array([x1,y1]) , np.array([x1,y1]) + np.array([d*sin(si),-d*cos(si)]) ],
								 			[  np.array([x2,y2]) - np.array([d*sin(si),-d*cos(si)]) , np.array([x2,y2]) , np.array([x2,y2]) + np.array([d*sin(si),-d*cos(si)]) ],
										    [  np.array([x3,y3]) - np.array([d*sin(si),-d*cos(si)]) , np.array([x3,y3]) , np.array([x3,y3]) + np.array([d*sin(si),-d*cos(si)]) ] ]), decimals = 3)	
	
	mat_wrt_global_A = copy.deepcopy(mat_wrt_global)
	mat_wrt_global_B = copy.deepcopy(mat_wrt_global)
	mat_wrt_global_C = copy.deepcopy(mat_wrt_global)
	if kk == 1:
		if 0.0 in P:
			for i in range(len(beta)):
				if beta[i][2] == 0.0:		
					if len(target_loc.shape) == 1:
						mat_wrt_global_A[beta[i][0]][beta[i][1]] = target_loc[0:2]
					else:
						mat_wrt_global_A[beta[i][0]][beta[i][1]] = [target_loc[ np.where(target_capture == 0.0)[0][0] ][ 0 ], target_loc[ np.where(target_capture == 0.0)[0][0] ][ 1 ]] 			 

		if 1.0 in P:		
			for i in range(len(beta)):
				if beta[i][2] == 1.0:
					if len(target_loc.shape) == 1:
						mat_wrt_global_B[beta[i][0]][beta[i][1]] = target_loc[0:2]
					else:
						mat_wrt_global_B[beta[i][0]][beta[i][1]] = [target_loc[ np.where(target_capture == 1.0)[0][0] ][ 0 ], target_loc[ np.where(target_capture == 1.0)[0][0] ][ 1 ]] 			 
		if 2.0 in P:		
			for i in range(len(beta)):
				if beta[i][2] == 2.0:
					if len(target_loc.shape) == 1:
						mat_wrt_global_C[beta[i][0]][beta[i][1]] = target_loc[0:2]
					else:
						mat_wrt_global_C[beta[i][0]][beta[i][1]] = [target_loc[ np.where(target_capture == 2.0)[0][0] ][ 0 ], target_loc[ np.where(target_capture == 2.0)[0][0] ][ 1 ]] 			 
	return mat_wrt_global,mat_wrt_global_A,mat_wrt_global_B,mat_wrt_global_C


def final_local_wrt_global(target_n,final_heading_angle):						## the position according to the global frame is assigned to the local coordinates
	d = 5 	# distance between two adgecent nodes
	#global final_mat_wrt_global
	x,y = target_n
	si = final_heading_angle 
	
	x1,y1 = np.array([x,y]) + np.array([d*cos(si),d*sin(si)])
	x2,y2 = np.array([x,y]) 
	x3,y3 = np.array([x,y]) - np.array([d*cos(si),d*sin(si)])

	final_mat_wrt_global  = np.around(np.array([  [  np.array([x1,y1]) - np.array([d*sin(si),-d*cos(si)]) , np.array([x1,y1]) , np.array([x1,y1]) + np.array([d*sin(si),-d*cos(si)]) ],
								 				  [  np.array([x2,y2]) - np.array([d*sin(si),-d*cos(si)]) , np.array([x2,y2]) , np.array([x2,y2]) + np.array([d*sin(si),-d*cos(si)]) ],
										    	  [  np.array([x3,y3]) - np.array([d*sin(si),-d*cos(si)]) , np.array([x3,y3]) , np.array([x3,y3]) + np.array([d*sin(si),-d*cos(si)]) ] ]), decimals = 3)
											
	return final_mat_wrt_global			
	
def check_if_reached(matrix_pos,tar):										##This funtion is finally aimed towards deciding the final position of the matrix required and if reached or not
	global ckr
	dist = sqrt((matrix_pos[0] - tar[0])**2 + (matrix_pos[1] - tar[1])**2)
	if dist <= 0.1:
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
	targ_a = mat_wrt_global_A[pos[0][0]][pos[0][1]]
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
	
## Navigation of robot C 
def error_calculation_ROBOT_C(pos,mat_wrt_global):
	global err_C
	targ_c = mat_wrt_global_C[pos[2][0]][pos[2][1]]
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
	targ_b = mat_wrt_global_B[pos[1][0]][pos[1][1]]
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
	global Q,N_T, something, P , beta   	
	Q = [] # distance of target from the nodes
	P = []	
	N_T = [] #distance of robots from the target	
	beta = []
	gamma = []
	xyz = [robot_A_pos,robot_B_pos,robot_C_pos]
	# T = []
	# T.append(alpha)
	T = alpha
	T_dash = copy.deepcopy(T)
	z = target_location.shape
	#T = [[0,0],[0,0],[0,0]]
	if len(target_location.shape) == 1:		
		P.append(target_location[2])
		for p in range(3):
			for q in range(3):			
				dist1 = sqrt((x_mat_wrt_global[p][q][0] - target_location[0])**2 + (x_mat_wrt_global[p][q][1] - target_location[1])**2)								
				Q.append(dist1)
			
	else:				
		for i in range(z[0]):
			P.append(target_location[i][2])		
			for p in range(3):
				for q in range(3):			
					dist1 = sqrt((x_mat_wrt_global[p][q][0] - target_location[i][0])**2 + (x_mat_wrt_global[p][q][1] - target_location[i][1])**2)				
					#dist2 = sqrt((pose[p][q][0] - target_location[0])**2 + (pose[p][q][1] - target_location[1])**2)
					Q.append(dist1)
					#N_T.append(dist2)
	
	if len(target_location.shape) == 1:
		for j in range(n):
			dist2 = sqrt((xyz[j][0] - target_location[0])**2 + (xyz[j][1] - target_location[1])**2)	
			N_T.append(dist2)	
			#print(N_T," Distance of robot from target")			
	else:
		for k in range(z[0]):
			for j in range(n):
				dist2 = sqrt((xyz[j][0] - target_location[k][0])**2 + (xyz[j][1] - target_location[k][1])**2)	
				N_T.append(dist2)			
				#print(Q)
	
	
	if len(Q) > 0:
		if len(target_location.shape) == 1:
			Q = np.around(Q,decimals = 2)	
			Q = Q.reshape(1,9)
		else:	
			Q = np.around(Q,decimals = 2)	
			Q = Q.reshape(z[0],9)
	
			
			
	# if len(target_location.shape) == 1:
	# 	a = 0
	# 	x = [0,1,2,3,4,5,6,7,8]
	# 	q = Q[:][a].reshape(3,3)
	# 	something = [np.where(q == np.amin(q))[0][0],np.where(q == np.amin(q))[1][0]]
	# 	#print(something,"something")
	# 	T_dash[N_T.index(min(N_T))]	=  copy.deepcopy(something)
	# 	for k in range(n):
	# 		if k == N_T.index(min(N_T)):
	# 			continue
	# 		else:
	# 			if 3*T_dash[N_T.index(min(N_T))][0] + T_dash[N_T.index(min(N_T))][1] == 3*T_dash[k][0] + T_dash[k][1]:
	# 				for l in range(n):
	# 					if l == k:
	# 						continue
	# 					else:
	# 						if 3*T_dash[l][0] + T_dash[l][1] in x:
	# 							x.remove(3*T_dash[l][0] + T_dash[l][1])
	# 				y = random.choice(x)
	# 				T_dash[k][0] = int(y/3)
	# 				T_dash[k][1] = int(y%3)
	# else:
	# 	for a in range (z[0]):		
	# 		a = 0
	# 		x = [0,1,2,3,4,5,6,7,8]
	# 		q = Q[:][a].reshape(3,3)
	# 		something = [np.where(q == np.amin(q))[0][0],np.where(q == np.amin(q))[1][0]]
	# 		T_dash[N_T.index(min(N_T))]	=  copy.deepcopy(something)
	# 		for k in range(n):
	# 			if k == N_T.index(min(N_T)):
	# 				continue
	# 			else:
	# 				if 3*T_dash[N_T.index(min(N_T))][0] + T_dash[N_T.index(min(N_T))][1] == 3*T_dash[k][0] + T_dash[k][1]:
	# 					for l in range(n):
	# 						if l == k:
	# 							continue
	# 						else:
	# 							if 3*T_dash[l][0] + T_dash[l][1] in x:
	# 								x.remove(3*T_dash[l][0] + T_dash[l][1])
	# 					y = random.choice(x)
	# 					T_dash[k][0] = int(y/3)
	# 					T_dash[k][1] = int(y%3)
						
						
	if 0.0 in P:
		x = [0,1,2,3,4,5,6,7,8]
		q1 = Q[:][P.index(0.0)].reshape(3,3)
		T1 = np.where(q1 == np.amin(q1))
		T[0][0] = T1[0][0]
		T[0][1] = T1[1][0]
		#something = [T[0][0],T[0][1]]
		beta.append([T1[0][0],T1[1][0],0.0])
		for k in range(n):
			if k == 0:
				continue
			else:
				if 3*T[0][0] + T[0][1] == 3*T[k][0] + T[k][1]:
					for l in range(n):
						if l == k:
							continue
						else:
							if 3*T[l][0] + T[l][1] in x:
								x.remove(3*T[l][0] + T[l][1])
					y = random.choice(x)
					T[k][0] = int(y/3)
					T[k][1] = int(y%3)
					
	if 1.0 in P:
		x = [0,1,2,3,4,5,6,7,8]
		q2 = Q[:][P.index(1.0)].reshape(3,3)
		T2 = np.where(q2 == np.amin(q2))
		T[1][0] = T2[0][0]
		T[1][1] = T2[1][0]
		#something = [T[1][0],T[0][1]]
		beta.append([T2[0][0],T2[1][0],1.0])	
		for k in range(n):
			if k == 1:
				continue
			else:
				if 3*T[1][0] + T[1][1] == 3*T[k][0] + T[k][1]:
					for l in range(n):
						if l == k:
							continue
						else:	
							if 3*T[l][0] + T[l][1] in x:
								x.remove(3*T[l][0] + T[l][1])
					y = random.choice(x)
					T[k][0] = int(y/3)
					T[k][1] = int(y%3)
	if 2.0 in P:
		x = [0,1,2,3,4,5,6,7,8]	
		q3 = Q[:][P.index(2.0)].reshape(3,3)	
		T3 = np.where(q3 == np.amin(q3))
		T[2][0] = T3[0][0]
		T[2][1] = T3[1][0]
		beta.append([T3[0][0],T3[1][0],2.0])
		for k in range(n):
			if k == 2:
				continue
			else:
				if 3*T[2][0] + T[2][1] == 3*T[k][0] + T[k][1]:
					print("--------------In there --------------", k, T[k][0],T[k][1])
					for l in range(n):
						if l == k:
							continue
						else:	
							if 3*T[l][0] + T[l][1] in x:
								x.remove(3*T[l][0] + T[l][1])
					y = random.choice(x)
					T[k][0] = int(y/3)
					T[k][1] = int(y%3)
	
	# for m in range (len(target_location.shape)):				
	# 	T_dash[N_T.index(min(N_T))]	
	
	return T, beta			
				
	

def payoff_compute(tar,pos, prev_pos):       ## Calculate the pay offs for each robot	
	global pof
	pof = []
		
	for i in range(n):
		c1 = c2 = c3 =c4 = 0
		k1 = k2 = k3 =k4 = 0
		z1 = z2 = z3 =z4 = 0
		est_pose = [[pos[i][0]+1, pos[i][1]],						# front     dec = 0
					[pos[i][0]-1, pos[i][1]],						# back			  1
					[pos[i][0], pos[i][1]-1],						# right           2
					[pos[i][0], pos[i][1]+1],						# left			  3
					[pos[i][0], pos[i][1]]]							# STAY            4
		
		for j in range (n):
			if j == i:
				continue
			else:
				if (est_pose[0] == pos[j]):
					c1 = 3
				if (est_pose[1] == pos[j]):
					c2 = 3
				if (est_pose[2] == pos[j]):
					c3 = 3
				if (est_pose[3] == pos[j]):
					c4 = 3

		if est_pose[0][0] < 0 or est_pose[0][1] > 2:
			k1 = 10
		if est_pose[1][0] < 0 or est_pose[1][1] > 2:
			k2 = 10
		if est_pose[2][0] < 0 or est_pose[2][1] > 2:
			k3 = 10
		if est_pose[3][0] < 0 or est_pose[3][1] > 2:
			k4 = 10
			
		if (est_pose[0] == prev_pos[i]):
			z1 = 3
		if (est_pose[1] == prev_pos[i]):
			z2 = 3
		if (est_pose[2] == prev_pos[i]):
			z3 = 3
		if (est_pose[3] == prev_pos[i]):
			z4 = 3	
	# k1 = 2*(abs(est_pose[0][0] - pos[i][0]) + abs(est_pose[0][1] - pos[i][1]))**2
		
		# k2 = 2*(abs(est_pose[1][0] - pos[i][0]) + abs(est_pose[1][1] - pos[i][1]))**2

		# k3 = 2*(abs(est_pose[2][0] - pos[i][0]) + abs(est_pose[2][1] - pos[i][1]))**2

		# k4 = 2*(abs(est_pose[3][0] - pos[i][0]) + abs(est_pose[3][1] - pos[i][1]))**2
			
			
		pof.append( [ targ_rwrd -  1*abs(est_pose[0][0] - tar[i][0]) - 1*abs(est_pose[0][1] - tar[i][1]) - c1 -k1 -z1,
	   			   	  targ_rwrd -  1*abs(est_pose[1][0] - tar[i][0]) - 1*abs(est_pose[1][1] - tar[i][1]) - c2 -k2-z2,
	   			      targ_rwrd -  1*abs(est_pose[2][0] - tar[i][0]) - 1*abs(est_pose[2][1] - tar[i][1]) - c3 -k3-z3,
	   			      targ_rwrd -  1*abs(est_pose[3][0] - tar[i][0]) - 1*abs(est_pose[3][1] - tar[i][1]) - c4 -k4-z4,
					  targ_rwrd -  3*abs(est_pose[4][0] - tar[i][0]) - 3*abs(est_pose[4][1] - tar[i][1]) ] )	
	return pof
			
def decision(pof,pos):				## deciding the next step of each robot
	x = []
	p = []
	est_pose = []
	global dec
	dec= []
	for i in range (n):
		#print(i,"i")
	#	x = pof[i].index(max(pof[i]))
		est_pose = []
		p = copy.deepcopy(pof[i])
		m = max(p)
		x = [k for k, j in enumerate(p) if j == m]
		
		#for i in range (n):			
		if 0 in x:
			if pos[i][0]+1 < 0 or pos[i][0]+1 > 2:
				x.remove(0)
			else:		
				est_pose.append((pos[i][0]+1, pos[i][1]))			
				#pos[i] = [pos[i][0]+1, pos[i][1]]
		if 1 in x:
			if pos[i][0]-1 < 0 or pos[i][0]-1 > 2:
				x.remove(1)
			else:				
				est_pose.append((pos[i][0]-1, pos[i][1]))
				#pos[i] = [pos[i][0]-1, pos[i][1]]
		if 2 in x:
			if pos[i][1]-1 < 0 or pos[i][1]-1 > 2:
				x.remove(2)
			else:	
				est_pose.append((pos[i][0], pos[i][1]-1))
				#pos[i] = [pos[i][0], pos[i][1]-1]
		if 3 in x:
			if pos[i][1]+1 < 0 or pos[i][1]+1 > 2:
				x.remove(3)
			else:
				est_pose.append((pos[i][0], pos[i][1]+1))
				#pos[i] = [pos[i][0], pos[i][1]+1]
		
		for j in range (n):
			#print(j,"j")
			est_pose_fr = []
			est = est_pose
			q1 = copy.deepcopy(pof[j])
			m1 = max(q1)
			x1 = [p for p, l in enumerate(q1) if l == m1]
			
			if j == i:
				#print("hello")
				x = [random.choice(x)]
				#print(x)
				break
			else:
				
				if dec[j] == [0]:							
					est_pose_fr.append((pos[j][0]+1, pos[j][1]))
					#print(est_pose_fr)			
						#pos[i] = [pos[i][0]+1, pos[i][1]]
				if dec[j] == [1]:
					est_pose_fr.append((pos[j][0]-1, pos[j][1]))
						#pos[j] = [pos[j][0]-1, pos[j][1]]
				if dec[j] == [2]:
					est_pose_fr.append((pos[j][0], pos[j][1]-1))
						#pos[j] = [pos[j][0], pos[j][1]-1]
				if dec[j] == [3]:
					est_pose_fr.append((pos[j][0], pos[j][1]+1))
				if dec[j] == [4]:
					est_pose_fr.append((pos[j][0], pos[j][1]))
					
				if len(list(set(est) & set(est_pose_fr))) >= 1:					
					#x.remove(pof[j].index(max(pof[j])))	
					print("hello")
					k = list(set(est) & set(est_pose_fr))
					est = list(set(est) - set(k))
									
					#x.remove(pof[j].index(max(pof[j])))
					
					if len(est) >= 1:
						print("est length is 1")
						x = [est_pose.index(est[0])]
						#break
					else:
						#est = random.choice(est)
						#x = [est_pose.index(est)]
						#p[pof[j].index(max(pof[j]))]=-10
						p[x[0]]=-10
						m = max(p)
						x = [i for i, j in enumerate(pof[i]) if j == m]
						x = [random.choice(x)]
				else:
					# if len(x) == 1:
					# 	break		
					# else:	
					x = [random.choice(x)]
						#x.pop(pof[i].index(max(pof[i])))												
		dec.append(x)	
		#print(dec)	
							  	
	return dec
	
def robot_pos_update_wrt_local(pose,dec):
	#global bot_conf
	# bot_conf = [['_','_','_'],
	# 			['_','_','_'],
	# 			['_','_','_']]
	for i in range (n):			
		if dec[i] == [0]:
			if pose[i][0]+1 >= 0 and pose[i][0]+1 < 3:
				pose[i] = [pose[i][0]+1, pose[i][1]]
		elif dec[i] == [1]:
			if pose[i][0]-1 >= 0 and pose[i][0]-1 < 3:
				pose[i] = [pose[i][0]-1, pose[i][1]]
		elif dec[i] == [2]:
			if pose[i][1]-1 >= 0 and pose[i][1]-1 < 3:
				pose[i] = [pose[i][0], pose[i][1]-1]
		elif dec[i] == [3]:
			if pose[i][1]+1 >= 0 and pose[i][1]+1 < 3:
				pose[i] = [pose[i][0], pose[i][1]+1]
			
	return pose

def check_if_next_step_reached(pose,mat_wrt_global,mat_wrt_global_A,mat_wrt_global_B,mat_wrt_global_C):					 #This function is to decide whether the required posee is reached or not
	global reconf_status
	#for robot A	
	d_A = sqrt((mat_wrt_global_A[pose[0][0]][pose[0][1]][0] - robot_A_pos[0])**2 + (mat_wrt_global_A[pose[0][0]][pose[0][1]][1] - robot_A_pos[1])**2)
	#print(d_A)
	if d_A <= 1.2:
		A_reached = 1
	else:
		A_reached = 0
	#for robot B
	d_B = sqrt((mat_wrt_global_B[pose[1][0]][pose[1][1]][0] - robot_B_pos[0])**2 + (mat_wrt_global_B[pose[1][0]][pose[1][1]][1] - robot_B_pos[1])**2)
	if d_B <= 1.2:
		B_reached = 1
	else:
		B_reached = 0
		
	d_C = sqrt((mat_wrt_global_C[pose[2][0]][pose[2][1]][0] - robot_C_pos[0])**2 + (mat_wrt_global_C[pose[2][0]][pose[2][1]][1] - robot_C_pos[1])**2)
	if d_C <= 1.2:
		C_reached = 1
	else:
		C_reached = 0	
		
	reconf_status = [A_reached,B_reached,C_reached]
	return reconf_status	
					
def target_check(target,matrix_pos):
	global target_loc
	target_loc = np.array([])
	#target_loc = np.append(target_loc,[ -1, -1 ,-1],axis = 0)
	kk = 0
	for i in range (3):
		for t in range (n):
			if target[i][t][2] == 0:
				if target[i][t][0] < matrix_pos[0] + 15 and target[i][t][0] > matrix_pos[0] - 15 and target[i][t][1] < matrix_pos[1] + 10 and target[i][t][1] > matrix_pos[1] - 10:
					if target_loc.shape[0] == 0:
						target_loc = np.append(target_loc, [ target[i][t][0], target[i][t][1] ,t],axis = 0)
					else:
						target_loc = np.append([target_loc],[[ target[i][t][0], target[i][t][1] ,t]],axis = 0)	
					kk = 1
		#print(kk)	
	return kk , target_loc	
						
def target_status_update():
	xyz = [robot_A_pos,robot_B_pos,robot_C_pos]
	for i in range (3):
		for k in range (3):
			#for j in range (n):
			if sqrt(((target[i][k][0]) - (xyz[k][0]))**2 + ((target[i][k][1]) - (xyz[k][1]))**2) <= 1.5:
				print("target_nullified")
				target[i][k][2] = 1
				target_nullified = 1
		
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
	prev_pose = copy.deepcopy(pose)
	for k in range(9):
		print("hello")
		ckr = 0
		kk = 0		
		mat_wrt_global,mat_wrt_global_A,mat_wrt_global_B,mat_wrt_global_C = initialize_local_wrt_global(matrix_pos,kk)
		check_if_next_step_reached(pose,mat_wrt_global,mat_wrt_global_A,mat_wrt_global_B,mat_wrt_global_C)

		#print(pos)
		while ckr == 0:
			prev_pos = pose
			target_status_update()
			kk , target_capture = target_check(target,matrix_pos)
			
			if kk == 1:
				error_calculation_targ(matrix_pos,target_capture)
			else:	
				error_calculation(matrix_pos,matrix_target_list[k+1])
							
			matrix_control_variable(vel_err,ang_err)		
			update_matrix_pose(u)
			#target_check(target,matrix_pos)
			
				
			if kk == 1:# and one_time_variable == 0:
				reconf_done = 0
				error_calculation_targ(matrix_pos,target_capture)
				print(kk,"-----------------------KK----------------------")
				si = matrix_pos[2]
				#print(pose)
				final_mat_wrt_global = final_local_wrt_global(targ_new,si)
				#print(pose)
				one_time_variable = 1
			 #print(final_local_wrt_global)
				qwerty = copy.deepcopy(pose)
				T, beta = local_matrix_target_for_reconf(target_capture,final_mat_wrt_global,qwerty)
				#target_nullified = 0
				#print(pose)
				#print(qwerty,"----------------------")
				#kk = 0
				print(T)
			
			
			mat_wrt_global,mat_wrt_global_A,mat_wrt_global_B,mat_wrt_global_C = initialize_local_wrt_global(matrix_pos,kk)
			if kk == 1 and reconf_status == [1,1,1]:
				#one_time_variable = 1
				if  T == pose:	
					print("true")				
					target_nullified = 1
					one_time_variable = 0
				print (reconf_status)
				pof = payoff_compute(T,pose,prev_pose)
				deci = decision(pof,pose)
				#print(pose)
				prev_pose = copy.deepcopy(pose)
				pose = robot_pos_update_wrt_local(pose,deci)
				reconf_status == [0,0,0]

			if kk == 0 and reconf_status == [1,1,1]:
				reconf_status == [0,0,0]
				
			## ROBOT NAVIGATION		
			# robot A navigation
			if reconf_status[0] == 0:
				error_calculation_ROBOT_A(pose,mat_wrt_global_A)
				control_variable_ROBOT_A(err_A)
				robot_A_navigation(u_A)
			# robot B navigation
			if reconf_status[1] == 0:
				error_calculation_ROBOT_B(pose,mat_wrt_global_B)
				control_variable_ROBOT_B(err_B)
				robot_B_navigation(u_B)
		
			if reconf_status[2] == 0:
				error_calculation_ROBOT_C(pose,mat_wrt_global_C)
				control_variable_ROBOT_C(err_C)
				robot_C_navigation(u_C)	
		###########
		#print(bot_conf)
			check_if_next_step_reached(pose,mat_wrt_global,mat_wrt_global_A,mat_wrt_global_B,mat_wrt_global_C)
			check_if_reached(matrix_pos,matrix_target_list[k+1])
			
		###### for plotting purpose
			x.append(matrix_pos[0])
			y.append(matrix_pos[1])
			x1.append(robot_A_pos[0])
			y1.append(robot_A_pos[1])
			x2.append(robot_B_pos[0])
			y2.append(robot_B_pos[1])
			x3.append(robot_C_pos[0])
			y3.append(robot_C_pos[1])
			if len(x1) >= 100:
				x1 = x1[-100:]
			if len(x2) >= 100:
				x2 = x2[-100:]
			if len(x3) >= 100:
				x3 = x3[-100:]
			if len(y1) >= 100:
				y1 = y1[-100:]
			if len(y2) >= 100:
				y2 = y2[-100:]
			if len(y3) >= 100:
				y3 = y3[-100:]
			if len(x) >= 100:
				x = x[-100:]
			if len(y) >= 100:
				y = y[-100:]																		
			#plt.plot(mat_wrt_global[0][0][0],mat_wrt_global[0][0][1],'r.')
			#plt.plot(mat_wrt_global[0][1][0],mat_wrt_global[0][1][1],'r.')
			# plt.plot(mat_wrt_global[0][2][0],mat_wrt_global[0][2][1],'r.')
			# plt.plot(mat_wrt_global[1][0][0],mat_wrt_global[1][0][1],'r.')
			plt.plot(mat_wrt_global[1][1][0],mat_wrt_global[1][1][1],'mX')
			# plt.plot(mat_wrt_global[1][2][0],mat_wrt_global[1][2][1],'r.')
			# plt.plot(mat_wrt_global[2][0][0],mat_wrt_global[2][0][1],'r.')
			# plt.plot(mat_wrt_global[2][1][0],mat_wrt_global[2][1][1],'r.')
			# plt.plot(mat_wrt_global[2][2][0],mat_wrt_global[2][2][1],'r.')
			plt.plot(x,y, color = (0.9,0.0,0.9,0.5))
			plt.plot(robot_A_pos[0],robot_A_pos[1],'ro')
			plt.plot(robot_B_pos[0],robot_B_pos[1],'go')
			plt.plot(robot_C_pos[0],robot_C_pos[1],'bo')
			plt.plot(target[0][0][0],target[0][0][1],'rx')
			plt.plot(target[0][1][0],target[0][1][1],'gx')
			plt.plot(target[0][2][0],target[0][2][1],'bx')
			plt.plot(target[1][0][0],target[1][0][1],'rx')
			plt.plot(target[1][1][0],target[1][1][1],'gx')
			plt.plot(target[1][2][0],target[1][2][1],'bx')
			plt.plot(target[2][0][0],target[2][0][1],'rx')
			plt.plot(target[2][1][0],target[2][1][1],'gx')
			plt.plot(target[2][2][0],target[2][2][1],'bx')
			# plt.plot(target[3][0],target[3][1],'gx')
			# plt.plot(target[4][0],target[4][1],'gx')
			# plt.plot(target[5][0],target[5][1],'gx')
			plt.plot(x1,y1,'r--')
			plt.plot(x2,y2,'g--')
			plt.plot(x3,y3,'b--')
			plt.axis([0,90,0,100])
			plt.xlabel('X-Axis (m)')
			plt.ylabel('Y-Axis (m)')
			#plt.axis('off')
			#plt.draw()
			plt.pause(0.01)
			#legend((line1, line2, line3), ('label1', 'label2', 'label3'))
			
			plt.clf()
			#print(vel_err,ang_err, "errors")
			#print(u_A,"control variables of A")
			sleep(0.0100)