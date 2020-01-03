import cv2
import numpy as np
import random

from collections import deque
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten , Input
from keras.optimizers import Adam
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten , Input
from keras.layers import Conv2D, Conv3D, MaxPooling2D
from keras import backend as K
from keras.models import Model
from keras import optimizers , metrics
from keras.layers.core import Activation
from keras.layers.merge import Concatenate
from keras.models import load_model


MATRIX_SIZE = 5
learning_rate = 0.1
img = cv2.imread('./img/whitespace.jpg')

#print (img.shape)
try:
    img.shape
    print("checked for shape".format(img.shape))
except AttributeError:
    print("shape not found")
	
img  = cv2.resize(img,(5,5))

x = 3	
y = 1

tar_x = 1
tar_y = 2


if x < tar_x:
	x1 = x
	x2 = tar_x
else:
	x1 = tar_x
	x2 = x

if y < tar_y:
	y1 = y
	y2 = tar_y
else:
	y1 = tar_y
	y2 = y

		
BLACK = [0,0,0]
img[x,y] = [0,0,255]
img[tar_x,tar_y] = [0,0,255]
#cv2.imshow('image',img)
crop_img = img[x1:(x2+1), y1:(y2+1)].copy()
#batch, channels, spatial_dim1, spatial_dim2, spatial_dim3
#img = np.reshape(img, [1,img.shape[0],img.shape[1],img.shape[2]])
constant= cv2.copyMakeBorder(crop_img,int(((5-(crop_img.shape[0]))%2) + (5-(crop_img.shape[0]))/2),int((5-(crop_img.shape[0]))/2),int(((5-(crop_img.shape[1]))%2) + (5-(crop_img.shape[1]))/2),int((5-(crop_img.shape[1]))/2),cv2.BORDER_CONSTANT,value=BLACK)

g_map = np.array(np.ones(shape=(MATRIX_SIZE, MATRIX_SIZE)))

g_map[x,y] = 5
g_map[tar_x,tar_y] = 10	

crop_gmap = g_map[x1:(x2+1) , y1:(y2+1)].copy()
new_g_map = np.array(np.zeros(shape=(MATRIX_SIZE, MATRIX_SIZE)))
new_g_map[2 - int((crop_gmap.shape[0] - 1)/2)  :   2 - int((crop_gmap.shape[0] - 1)/2) + crop_gmap.shape[0]
		 ,2 - int((crop_gmap.shape[1] - 1)/2)  :   2 - int((crop_gmap.shape[1] - 1)/2) + crop_gmap.shape[1]] = crop_gmap
		 
g_map = np.reshape(g_map, [1,5,5,1])

# action_size = 4
# PATCH_X=5
# PATCH_Y=5
# PATCH_Z=1
# NUM_CHANNEL=3

# def build_model():
#         # Neural Net for Deep-Q learning Model
# 	input_shape = (PATCH_Y,PATCH_X,NUM_CHANNEL)
# 	input_shape_B = (PATCH_Y,PATCH_X,1)
# 	inputA=Input(shape=(input_shape))
# 	inputB=Input(shape=(input_shape_B))
	
# 	x_1=Conv2D(4,(3,3),activation="relu")(inputA)
# 	x_2=Conv2D(2,(3,3),activation="relu")(inputB)
	
# 	x_1=Flatten()(x_1)
# 	x_2=Flatten()(x_2)
	
# 	x1=Concatenate(axis=-1)([x_1,x_2])
# 	#x1=Flatten()(x1)
# 	#x1=Dense(9)(inputA)
# 	#x1=Activation("relu")(x1)
# 	#x_2=Dense(9)(inputB)
# 	#x_2=Activation("relu")(x_2)
# 	#x1=Concatenate(axis=-1)([x_1,x_2])
# 	x1=Dense(16)(x1)
# 	x1=Activation("relu")(x1)
# 	x1=Dense(16)(x1)
# 	x1=Activation("relu")(x1)
# 	x1=Dense(action_size)(x1)
# 	x1=Activation("softmax")(x1)
# 	#x3=Activation("linear")(x3)
# 	model=Model(inputs=[inputA,inputB],outputs=x1)        
# 	#model.compile(loss='mse',
#      #                 optimizer=Adam(lr=learning_rate))
# 	return model
	
# model = build_model()	
# z = [img]
# z.append(g_map)
# action = model.predict(z)