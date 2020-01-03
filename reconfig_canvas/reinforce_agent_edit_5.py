## For single agent variale target training
import copy
import pylab
import numpy as np
from environment_edit_5 import Env
from keras.layers import Dense
from keras.optimizers import Adam
from keras.models import Sequential
from keras import backend as K
from keras.models import load_model
import random

EPISODES = 2500


# this is REINFORCE Agent for GridWorld
class ReinforceAgent:
    def __init__(self):
        self.load_model = False
        # actions which agent can do
        self.action_space = [0, 1, 2, 3, 4]
        # get size of state and action
        self.action_size = len(self.action_space)
        self.state_size = 8
        self.discount_factor = 0.99
        self.learning_rate = 0.001

        self.model = self.build_model()
        self.optimizer = self.optimizer()
        self.states, self.actions, self.rewards = [], [], []

        if self.load_model:
            self.model.load_weights('./save_model/reinforce_trained.h5')

    # state is input and probability of each action(policy) is output of network
    def build_model(self):
        model = Sequential()
        model.add(Dense(24, input_dim=self.state_size, activation='relu'))
        model.add(Dense(24, activation='relu'))
        model.add(Dense(self.action_size, activation='softmax'))
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
        train = K.function([self.model.input, action, discounted_rewards], [],
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
    def append_sample(self, state, action, reward):
        self.states.append(state[0])
        self.rewards.append(reward)
        act = np.zeros(self.action_size)
        act[action] = 1
        self.actions.append(act)

    # update policy neural network
    def train_model(self):
        discounted_rewards = np.float32(self.discount_rewards(self.rewards))
        discounted_rewards -= np.mean(discounted_rewards)
        discounted_rewards /= 0.00001 + np.std(discounted_rewards)

        self.optimizer([self.states, self.actions, discounted_rewards])
        self.states, self.actions, self.rewards = [], [], []


if __name__ == "__main__":
    env = Env()
    agent = ReinforceAgent()
    #agent2 = ReinforceAgent()
    #agent1.model.load_weights("./save_model/agent1_new.h5")
    #agent2.model.load_weights("./save_model/agent2_new.h5")
    
    global_step = 0
    scores, episodes = [], []
    global_step2 = 0
    scores2 = []
    
    score = 0 
    score2 = 0 
    done = [False,False]        
    target_x = random.sample(range(0,2),2)
    target_y = random.sample(range(0,2),2)
    state = env.reset()
    state1 = np.reshape(state[0], [1, 8])
    state2 = np.reshape(state[1], [1, 8])
    ckr = [0,0]
    for e in range(EPISODES):
        ckr = [0,0]
        score = 0
        score2 = 0
        
        #while 1:
        global_step += 1
        global_step2 += 1
            # get action for the current state and go one step in environment
        action1 = agent.get_action(state1)
        action2 = agent.get_action(state2)
            
        next_state, reward, done = env.step(action1,action2,target_x,target_y)
        next_state1 = np.reshape(next_state[0], [1, 8])
        next_state2 = np.reshape(next_state[1], [1, 8])
            
            #print(reward)
        agent.append_sample(state1, action1, reward[0] + reward[1])
        agent.append_sample(state2, action2, reward[0] + reward[1])
        score += reward[0]
        score2 += reward[1]
        state1 = copy.deepcopy(next_state1)
        state2 = copy.deepcopy(next_state2)

            #if done[0]:
                # update policy neural network for each episode
        agent.train_model()              
        scores.append(score)               
        episodes.append(e)
        score = round(score, 2)
        print("agent1", "episode:", e, "  score:", score, "  time_step:",
                      global_step)
                
        #score = 0      
        #target_x = random.sample(range(0,5),2)
        #state = env.reset(target_x,target_y)
        #ckr[0] = 1
        #done[0] = False
                
        #    if done[1]:  
        #agent2.train_model()
        scores2.append(score2)
        #episodes.append(e)
        score2 = round(score2, 2)
        print("agent2","episode:", e, "  score:", score2, "  time_step:",
                      global_step2)
            #     target_y = random.sample(range(0,5),2)  
            #     state = env.reset(target_x,target_y)
            #     ckr[1] = 1
            #     score2 = 0
            #     done[1] = False
            
            # if ckr == [1,1]:
            #     print("hello")
            #     break            

        if e % 100 == 0:
            #pylab.plot(episodes, scores, 'b')
            #pylab.savefig("./save_graph/reinforce.png")
            agent.model.save_weights("./save_model/hope.h5")
            #agent2.model.save_weights("./save_model/agent2_new2.h5")