import time
import numpy as np
import tkinter as tk
from PIL import ImageTk, Image
import random

PhotoImage = ImageTk.PhotoImage

UNIT = 50  # pixels
HEIGHT = 2  # grid height
WIDTH = 2  # grid width

np.random.seed(1)


class Env(tk.Tk):
    def __init__(self):
        super(Env, self).__init__()
        self.action_space = ['u', 'd', 'l', 'r']
        self.action_size = len(self.action_space)
        self.title('Reinforce')
        self.geometry('{0}x{1}'.format(HEIGHT * UNIT, HEIGHT * UNIT))
        self.shapes = self.load_images()
        self.canvas = self._build_canvas()
        self.counter = 0
        self.rewards = []
        self.rewards2 = []
        self.target_1 = []
        self.target_2 = []
        self.goal = []
        # obstacle
        #self.set_reward([0, 1], -1)
        #self.set_reward([1, 2], -1)
        #self.set_reward([2, 3], -1)
        # #goal
        #self.set_reward([4, 4], 1)

    def _build_canvas(self):
        canvas = tk.Canvas(self, bg='white',
                           height=HEIGHT * UNIT,
                           width=WIDTH * UNIT)
        # create grids
        for c in range(0, WIDTH * UNIT, UNIT):  # 0~400 by 80
            x0, y0, x1, y1 = c, 0, c, HEIGHT * UNIT
            canvas.create_line(x0, y0, x1, y1)
        for r in range(0, HEIGHT * UNIT, UNIT):  # 0~400 by 80
            x0, y0, x1, y1 = 0, r, HEIGHT * UNIT, r
            canvas.create_line(x0, y0, x1, y1)

        self.rewards = []
        self.rewards2 = []
        self.goal = []
        # add image to canvas
        x1, y1 = (UNIT/2), (UNIT/2)
        x2, y2 = (UNIT/2), 3*(UNIT/2)
		
        self.rectangle = canvas.create_image(x1, y1, image=self.shapes[0])
        self.rectangle2 = canvas.create_image(x2, y2, image=self.shapes[1])

        # pack all`
        canvas.pack()
        #PhotoImage(master = canvas, width = WIDTH, height = HEIGHT)
        return canvas

    def load_images(self):
        rectangle = PhotoImage(
            Image.open("./img/rectangle.png").resize((30, 30)))
        rectangle2 = PhotoImage(
            Image.open("./img/rectangle2.png").resize((30, 30)))    
        triangle = PhotoImage(
            Image.open("./img/triangle.png").resize((30, 30)))
        circle = PhotoImage(
            Image.open("./img/circle1.png").resize((30, 30)))
        circle2 = PhotoImage(
            Image.open("./img/circle2.png").resize((30, 30)))    

        return rectangle, rectangle2, circle, circle2

    def reset_reward(self,target_x,target_y):

        for reward in self.rewards:
            if reward['reward'] > 0:
                self.canvas.delete(reward['figure'])
          
        for reward in self.rewards2:
            if reward['reward'] > 0:
                self.canvas.delete(reward['figure'])    

        self.rewards.clear()
        self.rewards2.clear()
        self.goal.clear()
        #self.set_reward([0, 1], -1)
        #self.set_reward([1, 2], -1)
        #self.set_reward([2, 3], -1)
        #self.target1 = self.canvas.create_image((UNIT * target_x[0]) + UNIT / 2,
        #                                               (UNIT * target_x[1]) + UNIT / 2,
        #                                               image=self.shapes[2])
        #self.target2 = self.canvas.create_image((UNIT * target_y[0]) + UNIT / 2,
        #                                               (UNIT * target_y[1]) + UNIT / 2,
        #                                               image=self.shapes[3])                                               
        # #goal
        self.rewards.append(self.set_reward([0,1], 2,1))
        self.rewards2.append(self.set_reward([1,1], 2,2))
        
        self.rewards.append(self.set_reward([1,0], 1,1))
        self.rewards.append(self.set_reward([0,1], 1,1))
        
        self.rewards2.append(self.set_reward([0,0], 1,2))
        self.rewards2.append(self.set_reward([1,1], 1,2))


    def set_reward(self, state, reward,i):
        state = [int(state[0]), int(state[1])]
        location = [self.coords_to_state(self.canvas.coords(self.rectangle)) , self.coords_to_state(self.canvas.coords(self.rectangle2))]            
            
        #location2 = self.coords_to_state(self.canvas.coords(self.rectangle2))
        x = int(state[0])
        y = int(state[1])
        temp = {}
        if reward > 0:
            temp['reward'] = reward
            # if i == 2:
            #     temp['figure'] = self.target1
            # elif i == 3:
            #     temp['figure'] = self.target2
                    
            self.goal.append(temp['figure'])
            temp['state'] = state


        elif reward < 0:
            temp['directionx'] = state[0] - location[i][0]
            temp['directiony'] = state[1] - location[i][1]
            temp['reward'] = reward
            # if i == 1:
            #     temp['figure'] = self.rectangle2
            # elif i == 0:
            #     temp['figure'] = self.rectangle  
                
            temp['state'] = location[i]          
            #temp['figure'] = self.canvas.create_image((UNIT * x) + UNIT / 2,
            #                                          (UNIT * y) + UNIT / 2,
            #                                          image=self.shapes[1])

        temp['coords'] = self.canvas.coords(temp['figure'])
        #temp['state'] = state
        
        return temp
        #self.rewards.append(temp)

    # new methods

    def check_if_reward(self, state, rew):
        check_list = dict()
        check_list['if_goal'] = False
        rewards = 0

        for reward in rew:
            if reward['state'] == state:
                rewards += reward['reward']
                if reward['reward'] > 0:
                    check_list['if_goal'] = True

        check_list['rewards'] = rewards

        return check_list

    def coords_to_state(self, coords):
        x = int((coords[0] - UNIT / 2) / UNIT)
        y = int((coords[1] - UNIT / 2) / UNIT)
        return [x, y]

    def reset(self):
        self.update()
        #x, y = self.canvas.coords(self.rectangle)
        self.canvas.move(self.rectangle, - 50, - 50)
        self.canvas.move(self.rectangle2, - 50, + 50)
        # return observation
        #tar = random.sample(range(0,25),1)
        #target = [int(tar[0]/5),tar[0]%5]
        #target_x = random.sample(range(0,5),2)
        #target_y = random.sample(range(0,5),2)
        #print("hello")
        self.reset_reward()
        return self.get_state() 

    def step(self, action1,action2,target_x,target_y):
        self.counter += 1
        self.render()
        self.rewards.clear()
        self.rewards2.clear()

        #if self.counter % 2 == 1:
        #self.rewards = self.move_rewards(self.rewards)
        #self.rewards2 = self.move_rewards(self.rewards2)

        next_coords = self.move(self.rectangle, action1 , self.rectangle2 , action2)
        #self.rewards = self.move_rewards(self.rewards)
        #self.rewards2 = self.move_rewards(self.rewards2)
        self.rewards.append(self.set_reward([target_x[0], target_x[1]], 1,2))
        self.rewards2.append(self.set_reward([target_y[0], target_y[1]], 1,3))
        self.rewards.append(self.set_reward([target_y[0], target_y[1]], -1,1))
        self.rewards2.append(self.set_reward([target_x[0], target_x[1]], -1,0))
        check1 = self.check_if_reward(self.coords_to_state(next_coords[0]),self.rewards)
        check2 = self.check_if_reward(self.coords_to_state(next_coords[1]),self.rewards2)
        done1 = check1['if_goal']
        done2 = check2['if_goal']
        reward1 = check1['rewards']
        reward2 = check2['rewards']
        reward1 -= 0.1
        reward2 -= 0.1
        self.canvas.tag_raise(self.rectangle)
        self.canvas.tag_raise(self.rectangle2)
        
        reward = [reward1,reward2]
        done = [done1, done2]

        s_ = self.get_state()

        return s_, reward, done

    def get_state(self):

        location = self.coords_to_state(self.canvas.coords(self.rectangle))
        location2 = self.coords_to_state(self.canvas.coords(self.rectangle2))
        agent_x = location[0]
        agent_y = location[1]
        
        agent2_x = location2[0]
        agent2_y = location2[1]

        states = list()
        states2 = list()

        # locations.append(agent_x)
        # locations.append(agent_y)

        for reward in self.rewards:
            reward_location = reward['state']
            states.append(reward_location[0] - agent_x)
            states.append(reward_location[1] - agent_y)
            if reward['reward'] < 0:
                states.append(-1)
                states.append(reward['directionx'])
                states.append(reward['directiony'])
            else:
                states.append(1)

        for reward in self.rewards2:
            reward_location = reward['state']
            states2.append(reward_location[0] - agent2_x)
            states2.append(reward_location[1] - agent2_y)
            if reward['reward'] < 0:
                states2.append(-1)
                states2.append(reward['directionx'])
                states2.append(reward['directiony'])
            else:
                states2.append(1)

        return states,states2

    def move_rewards(self,rewards):
        new_rewards = []
        for temp in rewards:
            if temp['reward'] > 0:
                new_rewards.append(temp)
                continue
            temp['coords'] = self.canvas.coords(temp)
            temp['state'] = self.coords_to_state(temp['coords'])
            new_rewards.append(temp)
        return new_rewards

    def move_const(self, target):

        s = self.canvas.coords(target['figure'])

        base_action = np.array([0, 0])

        # if s[0] == (WIDTH - 1) * UNIT + UNIT / 2:
        #     target['direction'] = 1
        # elif s[0] == UNIT / 2:
        #     target['direction'] = -1

        # if target['direction'] == -1:
        #     base_action[0] += UNIT
        # elif target['direction'] == 1:
        #     base_action[0] -= UNIT

        if (target['figure'] is not self.rectangle
           and s == [(WIDTH - 1) * UNIT, (HEIGHT - 1) * UNIT]):
            base_action = np.array([0, 0])

        self.canvas.move(target['figure'], base_action[0], base_action[1])

        s_ = self.canvas.coords(target['figure'])

        return s_

    def move(self, target, action , target2, action2):
        s = self.canvas.coords(target)
        s2 = self.canvas.coords(target2)
        
        #cord1 = np.array([s[0], s[1]])
        #cord2 = np.array([s2[0], s2[1]])

        base_action = np.array([0, 0])
        base_action2 = np.array([0, 0])

        if action == 0:  # up
            if s[1] > UNIT:
                base_action[1] -= UNIT
        elif action == 1:  # down
            if s[1] < (HEIGHT - 1) * UNIT:
                base_action[1] += UNIT
        elif action == 2:  # right
            if s[0] < (WIDTH - 1) * UNIT:
                base_action[0] += UNIT
        elif action == 3:  # left
            if s[0] > UNIT:
                base_action[0] -= UNIT
                
        if action2 == 0:  # up
            if s2[1] > UNIT:
                base_action2[1] -= UNIT
        elif action2 == 1:  # down
            if s2[1] < (HEIGHT - 1) * UNIT:
                base_action2[1] += UNIT
        elif action2 == 2:  # right
            if s2[0] < (WIDTH - 1) * UNIT:
                base_action2[0] += UNIT
        elif action2 == 3:  # left
            if s2[0] > UNIT:
                base_action2[0] -= UNIT   
                
        cord1 =  [ s[0] + base_action[0] , s[1] + base_action[1] ]
        cord2 =  [ s2[0] + base_action2[0] , s2[1] + base_action2[1] ]
        
        if cord1 == cord2:
            print("crash---------------------------------------------------------------------")
            #base_action = np.array([0, 0])
            #base_action2 = np.array([0, 0])                        

        self.canvas.move(target, base_action[0], base_action[1])
        self.canvas.move(target2, base_action2[0], base_action2[1])

        s_1 = self.canvas.coords(target)
        s_2 = self.canvas.coords(target2)

        return s_1, s_2

    def render(self):
        time.sleep(0.07)
        self.update()