## For single agent variale target training
import copy
import pylab
import numpy as np
from environment import Env
from keras import backend as K
from keras.models import load_model
from collections import deque
from keras.layers import Dense, Dropout, Flatten , Input
from keras.optimizers import Adam
from keras.models import Sequential
from keras.layers import Conv2D, Conv3D, MaxPooling2D
from keras.models import Model
from keras import optimizers , metrics
from keras.layers.core import Activation
from keras.layers.merge import Concatenate
from keras.models import load_model
import cv2

EPISODES = 2500
PATCH_X=5
PATCH_Y=5
NUM_CHANNEL=3

# this is REINFORCE Agent for GridWorld
class ReinforceAgent:
    def __init__(self):
        self.load_model = False
        # actions which agent can do
        self.action_space = [0, 1, 2, 3, 4]
        # get size of state and action
        self.action_size = len(self.action_space)
        self.input_shape_A = (5,5,3)
        self.input_shape_B = (5,5,1)
        self.state_size = 3
        self.discount_factor = 0.99
        self.learning_rate = 0.01

        self.model = self.build_model()
        self.Optimizer = self.optimizer()
        self.states1, self.states2, self.actions, self.rewards = [], [], [], []

        # if self.load_model:
        #     self.model.load_weights('./save_model/reinforce_trained.h5')

    # state is input and probability of each action(policy) is output of network
    def build_model(self):
        inputA=Input(shape=(self.input_shape_A))       
        x_1=Conv2D(2,(1,1),activation="relu")(inputA)
        x1=Conv2D(1,(3,3),activation="relu")(x_1)
        x1=Flatten()(x1)
        
        inputB=Input(shape=(self.input_shape_B))
        x_2=Conv2D(2,(1,1),activation="relu")(inputB)        
        x2=Conv2D(1,(3,3),activation="relu")(x_2)         
        x2=Flatten()(x2)
        
        x=Concatenate(axis=-1)([x1,x2])
        x=Dense(24)(x)
        x=Activation("relu")(x)
        x=Dense(16)(x)
        x=Activation("softmax")(x)
        x=Dense(self.action_size)(x)
        x=Activation("softmax")(x)
        model=Model(inputs=[inputA,inputB],outputs=x)    
        # model = Sequential()
        # model.add(Dense(24, input_dim=self.state_size, activation='relu'))
        # model.add(Dense(24, activation='relu'))
        # model.add(Dense(self.action_size, activation='softmax'))
        model.summary()
        return model

    # create error function and training function to update policy network
    def optimizer(self):
        action = K.placeholder(shape=[None, 5])
        discounted_rewards = K.placeholder(shape=[None, ])

        # Calculate cross entropy error function
        action_prob = K.sum(action * self.model.output, axis=1)
        cross_entropy = K.log(action_prob) * discounted_rewards
        loss = -K.sum(cross_entropy)

        # create training function
        optimizer = Adam(lr=self.learning_rate)
        updates = optimizer.get_updates(self.model.trainable_weights, [],
                                        loss)
        train = K.function([self.model.input[0],self.model.input[1], action, discounted_rewards], [],
                           updates=updates)

        return train

    # get action from policy network
    def get_action(self, state):
        policy = self.model.predict(state)[0]
        #print(policy)
        return np.random.choice(self.action_size, 1, p=policy)[0]

    # calculate discounted rewards
    def discount_rewards(self, rewards):
        discounted_rewards = np.zeros_like(rewards)
        running_add = 0
        for t in reversed(range(0, len(rewards))):
            running_add = running_add * self.discount_factor + rewards[t]
            discounted_rewards[t] = running_add
        return discounted_rewards

    # save states, actions and rewards for an episode
    def append_sample(self, state1,state2, action, reward):
        self.states1.append(state1[0])
        self.states2.append(state2[0])
        self.rewards.append(reward)
        act = np.zeros(self.action_size)
        act[action] = 1
        self.actions.append(act)

    # update policy neural network
    def train_model(self):
        discounted_rewards = np.float32(self.discount_rewards(self.rewards))
        discounted_rewards -= np.mean(discounted_rewards)
        discounted_rewards /= 0.00001 + np.std(discounted_rewards)

        self.Optimizer([self.states1,self.states2, self.actions, discounted_rewards])
        self.states1, self.states2, self.actions, self.rewards = [], [], [], []


if __name__ == "__main__":
    env = Env()
    agent = ReinforceAgent()

    global_step = 0
    scores, episodes = [], []

    for e in range(EPISODES):
        done = False
        score = 0
        #agent.model.load_model('qwerty_1.h5')
        # fresh env
        #tar = random.sample(range(0,25),1)
        #target = [int(tar[0]/5),tar[0]%5]
        img,g_map = env.reset()
        cv2.imshow('image',img)
        #state = np.reshape(state, [1, 3])
        #img = state[0]
        #g_map = state[1]
        img = np.reshape(img, [1,img.shape[0],img.shape[1],img.shape[2]])
        g_map = np.reshape(g_map, [1,5,5,1])
        #state = [img,g_map]
        
        while not done:
            global_step += 1
            # get action for the current state and go one step in environment
            action = agent.get_action([img,g_map])
            next_state, reward, done = env.step(action)
            img = next_state[0]
            cv2.imshow('image',img)
            g_map = next_state[1]
            img = np.reshape(img, [1,img.shape[0],img.shape[1],img.shape[2]])
            g_map = np.reshape(g_map, [1,5,5,1])
            #print(g_map)
            #next_state = [img,g_map]
            #next_state = np.reshape(next_state, [1, 3])

            agent.append_sample(img,g_map, action, reward)
            score += reward
            img = copy.deepcopy(img)
            g_map = copy.deepcopy(g_map)

            if done:
                # update policy neural network for each episode
                agent.train_model()
                scores.append(score)
                episodes.append(e)
                score = round(score, 2)
                print("episode:", e, "  score:", score, "  time_step:",
                      global_step)

        if e % 100 == 0:
            pylab.plot(episodes, scores, 'b')
            pylab.savefig("./save_graph/reinforce.png")
            agent.model.save_weights("./save_model/reinforce.h5")