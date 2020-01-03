## new file uses front right left right as strategies not the coordinates## this is the python code for formation reconfiguration using game theory.
import numpy as np
import operator
import random
import copy

bot_conf = np.array([['_','_','_'],
			  		 ['_','_','_'],
					 ['A','B','C']])
# bot_conf = np.array([['A','B'],
# 			  		 ['_','_']])
bot_list = ['A','B','C']
n = len(bot_list)
mat_size = [3,3]
tasks = np.array([['T1' ,'T3' ,'T2' ],
				  ['_','_','_'],
				  ['_' ,'_' ,'_' ]])
#tasks = np.array([['T2','_'],['_','T1']])
task_id = [[0,0],[0,2],[0,1]]
target_id = ['T1','T2','T3']
# A can do task T1 and B can do task T2 and so on ..
side_swap = 1
diag_swap = 3
targ_rwrd = 5
recof_flag = 1
ckr = 0
tar = []
pos = []
pof = []
dec = []

#est_pose = [[0,0],[0,1],[1,0],[1,1]]
#def graph_reconfig():
#def reconfigure(bot_conf,targets):
def target():							## Check the targets of required robot depending upon the task
	task = []	
	f = []	
	k = 0
	for i in range(n):
		task.append(np.where(tasks == target_id[i]))
		f.append([task[i][0][0],task[i][1][0]])
	return f
		
def pos_est(pos,dec):
	global bot_conf
	bot_conf = [['_','_','_'],
				['_','_','_'],
				['_','_','_']]
	for i in range (n):			
		if dec[i] == 0:
			if pos[i][0]+1 >= 0 and pos[i][0]+1 < 3:
				pos[i] = [pos[i][0]+1, pos[i][1]]
		elif dec[i] == 1:
			if pos[i][0]-1 >= 0 and pos[i][0]-1 < 3:
				pos[i] = [pos[i][0]-1, pos[i][1]]
		elif dec[i] == 2:
			if pos[i][1]-1 >= 0 and pos[i][1]-1 < 3:
				pos[i] = [pos[i][0], pos[i][1]-1]
		elif dec[i] == 3:
			if pos[i][1]+1 >= 0 and pos[i][1]+1 < 3:
				pos[i] = [pos[i][0], pos[i][1]+1]
			
		#x = bot_list[i]	
		bot_conf [pos[i][0]] [pos[i][1]] = bot_list[i]
		
	bot_conf = np.array(bot_conf)
	return pos, bot_conf									
			
def payoff_compute(tar,pos,t, prev_pos):       ## Calculate the pay offs for each robot	
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
		
		for j in range (n):		#swap
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
			k1 = 3
		if est_pose[1][0] < 0 or est_pose[1][1] > 2:
			k2 = 3
		if est_pose[2][0] < 0 or est_pose[2][1] > 2:
			k3 = 3
		if est_pose[3][0] < 0 or est_pose[3][1] > 2:
			k4 = 3
			
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
					  targ_rwrd -  3*abs(est_pose[4][0] - tar[i][0]) - 3*abs(est_pose[4][1] - tar[i][1]) -t/100 ] )	
	return pof
			
def decision(pof):				## deciding the next step of each robot
	x = []
	p = []
	est_pose = []
	global dec, action, estimate_pos, pof_dash
	dec= []
	action = []
	estimate_pos = copy.deepcopy(pos)

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


	# for i in range (n):
	# 	#print(i,"i")
	# #	x = pof[i].index(max(pof[i]))
	# 	est_pose = []
	# 	p = copy.deepcopy(pof[i])
	# 	m = max(p)
	# 	x = [k for k, j in enumerate(p) if j == m]
		
	# 	#for i in range (n):			
	# 	if 0 in x:
	# 		if pos[i][0]+1 < 0 or pos[i][0]+1 > 2:
	# 			x.remove(0)
	# 		else:		
	# 			est_pose.append((pos[i][0]+1, pos[i][1]))			
	# 			#pos[i] = [pos[i][0]+1, pos[i][1]]
	# 	if 1 in x:
	# 		if pos[i][0]-1 < 0 or pos[i][0]-1 > 2:
	# 			x.remove(1)
	# 		else:				
	# 			est_pose.append((pos[i][0]-1, pos[i][1]))
	# 			#pos[i] = [pos[i][0]-1, pos[i][1]]
	# 	if 2 in x:
	# 		if pos[i][1]-1 < 0 or pos[i][1]-1 > 2:
	# 			x.remove(2)
	# 		else:	
	# 			est_pose.append((pos[i][0], pos[i][1]-1))
	# 			#pos[i] = [pos[i][0], pos[i][1]-1]
	# 	if 3 in x:
	# 		if pos[i][1]+1 < 0 or pos[i][1]+1 > 2:
	# 			x.remove(3)
	# 		else:
	# 			est_pose.append((pos[i][0], pos[i][1]+1))
	# 			#pos[i] = [pos[i][0], pos[i][1]+1]
		
	# 	for j in range (n):
	# 		#print(j,"j")
	# 		est_pose_fr = []
	# 		est = est_pose
	# 		q1 = copy.deepcopy(pof[j])
	# 		m1 = max(q1)
	# 		x1 = [p for p, l in enumerate(q1) if l == m1]
			
	# 		if j == i:
	# 			#print("hello")
	# 			x = [random.choice(x)]
	# 			#print(x)
	# 			break
	# 		else:
				
	# 			if dec[j] == [0]:							
	# 				est_pose_fr.append((pos[j][0]+1, pos[j][1]))
	# 				#print(est_pose_fr)			
	# 					#pos[i] = [pos[i][0]+1, pos[i][1]]
	# 			if dec[j] == [1]:
	# 				est_pose_fr.append((pos[j][0]-1, pos[j][1]))
	# 					#pos[j] = [pos[j][0]-1, pos[j][1]]
	# 			if dec[j] == [2]:
	# 				est_pose_fr.append((pos[j][0], pos[j][1]-1))
	# 					#pos[j] = [pos[j][0], pos[j][1]-1]
	# 			if dec[j] == [3]:
	# 				est_pose_fr.append((pos[j][0], pos[j][1]+1))
	# 			if dec[j] == [4]:
	# 				est_pose_fr.append((pos[j][0], pos[j][1]))
					
	# 			if len(list(set(est) & set(est_pose_fr))) >= 1:					
	# 				#x.remove(pof[j].index(max(pof[j])))	
	# 				print("hello")
	# 				k = list(set(est) & set(est_pose_fr))
	# 				est = list(set(est) - set(k))
									
	# 				#x.remove(pof[j].index(max(pof[j])))
					
	# 				if len(est) >= 1:
	# 					print("est length is 1")
	# 					x = [est_pose.index(est[0])]
	# 					#break
	# 				else:
	# 					#est = random.choice(est)
	# 					#x = [est_pose.index(est)]
	# 					#p[pof[j].index(max(pof[j]))]=-10
	# 					p[x[0]]=-10
	# 					m = max(p)
	# 					x = [i for i, j in enumerate(pof[i]) if j == m]
	# 					x = [random.choice(x)]
	# 			else:
	# 				# if len(x) == 1:
	# 				# 	break		
	# 				# else:	
	# 				x = [random.choice(x)]
	# 					#x.pop(pof[i].index(max(pof[i])))												
	# 	dec.append(x)	
		#print(dec)	

	dec = copy.deepcopy(action)			  	
	return dec

def check_reach(task_id,dec):					## checking whether the required positions are achieved or not
	global ckr
	if task_id == pos:
		ckr = 1
	else:
		ckr = 0	
	return ckr
		
if __name__ == '__main__':
	#check_reach(task_id,dec)
	#while ckr == 0:
	Bot = []
	for i in range(n):
		Bot.append(	np.where( bot_conf == bot_list[i] ) )
		pos.append( [ Bot[i][0][0], Bot[i][1][0] ] )			
	tar = target()
	prev_pos =copy.deepcopy(pos)
	print(bot_conf)
	print ('------------------------------')
	t = 0
	while (ckr == 0):	
	#for i in range (10):			
		payoff_compute(tar,pos,t,prev_pos)
		print(pof)
		prev_pos =copy.deepcopy(pos)
		#print(pof, "pay offs")
		decision(pof)
		print(dec)
		#print(dec, "decisions")
		pos_est(pos,dec)
		check_reach(task_id,dec)
		#print(ckr)
		
		print (bot_conf)
		print ('------------------------------')
		t= t+1
		if prev_pos != pos:
			t = 0
		
		# #print (ckr)