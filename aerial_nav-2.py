import numpy as np
import matplotlib.pyplot as plt


import numpy as np
import gym
from random import random 
class Football:  # The class encapsulating the environment
    '''
    Actions [0 : Stand, 1 : Up, 2 : Right, 3 : Down, 4 : Left]
    These are the representing no.s for the mentioned actions
    '''

    def __init__(self, length=30, width=30, goalPositions=[15,29]):
        
        # The player start at random locations
        
        self.pA=[np.random.randint(30),np.random.randint(30)] 
        self.pO=[6,8]
            
        
        self.h = length   # Length of the Football Pitch    
        self.w = width    # Width of the Football Pitch
        goalPositions=[np.random.randint(30),np.random.randint(30)] 
        self.goalPositions = np.array(goalPositions)   # This means that the middle 4 positions at the right and left are the goals
        
     
        
        self.reward = 0                            # Initially the reward is 0
        
        self.observation=np.random.rand(6,)
        self.done = bool(0)                          # This stores whether the game needs to be restart with new position (in the case of a goal)

    def reset(self):
        self.done = bool(0)
        self.reward = 0
        
        self.pA = np.array([np.random.randint(self.h), np.random.randint(self.h)])
        #self.pA = np.array([15,0])
        #self.pO=[6,8]
        self.x_traj,self.y_traj=[],[]
        self.xo_traj,self.yo_traj=[],[]
        return np.array((*self.pA,(self.x_goal-self.pA[0]),(self.y_goal-self.pA[1]),self.theta,*self.pO)).astype(np.float32)

    def step(self, action):
        if self.done == bool(1):
          self.reset()
        self.move(first, action)                   # We chose the first player at random
        if self.done == bool(1):
          return self.observation, self.reward, self.done
        if not done:
            self.current_player_num = (self.current_player_num + 1) % 2   
        return self.observation,self.reward, self.done,{}

    def move(self, player, action):
        opponent = 1 - player
        
        newPosition = self.pA + self.actionToMove(action)
        
        if self.ballOwner is player and self.isInGoal(*newPosition) >= 0:
            self.done = bool(1)
            return 1 - self.isInGoal(*newPosition)
        # If it's in the board
        elif self.isInBoard(*newPosition):
            self.positions[player] = newPosition
        if(self.ballOwner!=0):
          self.reward=-1
        return -1



    def isInBoard(self, x, y):
        if(x<0 or x>(30)):
          return 0
        if(y<0 or y>(30)):
          return 0 
        return 1
        

    #def choosePlayer(self):
    #    return np.random.randint(0, 2)
    def render(self,mode="human"):
        

        plt.cla()
        plt.arrow(x_start, y_start, np.cos(theta_start),
                      np.sin(theta_start), color='r', width=0.1)
        plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                      np.sin(theta_goal), color='g', width=0.1)
        plot_vehicle(x, y, theta, x_traj, y_traj)

class modf_football(Football,gym.Env):
  def __init__(self, length=30, width=30, goalPositions=np.array([15, 29])):
    super().__init__()
    self.observation_space=gym.spaces.Box(low=-30, high=60,
                                        shape=(7,), dtype=np.float32)
    self.reward=0
    self.action_space = gym.spaces.Box(
            low=-2*np.pi, high=2*np.pi, shape=(1,), dtype=np.float32
        )
    self.name='Football'
    self.current_player_num=0
    self.observation=np.random.rand(6,)
    self.pA=np.array([np.random.randint(30),np.random.randint(30)]) 
    #self.pA=np.array([0,15]) 
    self.Kp_rho = 9
    self.dt=0.01
    self.Kp_alpha = 15
    self.Kp_beta = -3
    self.x_goal = np.random.randint(30)
    self.y_goal = np.random.randint(30)
    self.theta_goal = 0 
    self.theta_start = 2 * np.pi * random() - np.pi
    
    self.theta = self.theta_start

    self.x_diff = self.x_goal - self.pA[0]
    self.y_diff = self.y_goal - self.pA[1]

    self.x_traj, self.y_traj = [], []
    self.xo_traj, self.yo_traj = [], []
    self.x_start=self.pA[0]
    self.y_start=self.pA[1]
    self.rho = np.hypot(self.x_diff, self.y_diff)
  #modifying the step and move function to get the updated reward system
  def step(self, action):
        
        #print('action',action)
        if self.done == bool(1):
          self.reset()
        
      
        self.move(action)                   # We chose the first player at random
        if self.done == bool(1):
          return self.observation, self.reward, self.done,{}
        #print(type(self.reward))           
        return self.observation,self.reward, self.done,{}
  
  def move(self, action):
        self.x_diff = self.x_goal - self.pA[0]
        self.y_diff = self.y_goal - self.pA[1]
        self.rho=np.hypot(self.x_diff,self.y_diff)
        v = self.Kp_rho * self.rho
        #v=9
        #w = self.Kp_alpha * action[0] + self.Kp_beta * action[1]
        #if action[0] > np.pi / 2 or action[0] < -np.pi / 2:
        #    v = -v
        #elif (self.pA[0]==0 and self.pA[1]==29) or (self.pA[1]==0 and self.pA[0]==29):
        #  v=-v
        self.theta = action
        x = self.pA[0] +  v* np.cos(action) * self.dt
        y = self.pA[1] +  v* np.sin(action) * self.dt
        newPosition = np.array([x,y])
        self.x_traj.append(x)
        self.y_traj.append(y)
        self.xo_traj.append(self.pO[0])
        self.yo_traj.append(self.pO[1])

        
        if (self.pO[0]>self.pA[0]):
          self.pO[0]-=0.5
        elif (self.pO[0]<self.pA[0]):
          self.pO[0]+=0.5
        if (self.pO[1]>self.pA[1]):
          self.pO[1]-=0.5
        elif (self.pO[1]<self.pA[1]):
          self.pO[1]+=0.5
        
        self.pO[0]=max(self.pO[0],0)
        self.pO[0]=min(self.pO[0],29)
        self.pO[1]=max(self.pO[1],0)
        self.pO[1]=min(self.pO[1],29)
      

        if self.isInGoal(*newPosition) >= 0:
            
            self.done = bool(1)
            return 1 - self.isInGoal(*newPosition)
        # If it's in the board
        if self.isInBoard(*newPosition):
               
            self.reward =  -0.1 * (abs(self.pA[0]-self.x_goal)+ abs(self.pA[1]-self.y_goal)) #+ 0.01*(abs(newPosition[0]-7)+abs(newPosition[1]-7)) +0.01*(abs(newPosition[0]-5)+abs(newPosition[1]-17))+0.01*(abs(newPosition[0]-10)+abs(newPosition[1]-13))
            self.pA = newPosition

        
        self.observation=np.array((*self.pA,(15-self.pA[0]),(15-self.pA[1]),self.theta,*self.pO)).astype(np.float32)
        return -1
  def isInGoal(self, x, y):
     x_diff = self.x_goal - x
     y_diff = self.y_goal - y
     rho = np.hypot(x_diff, y_diff)
     if(rho<0.001):
       self.done=bool(1)
       self.reward=20  
     return -1
  def transformation_matrix(self,x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])
  def plot_vehicle(self,x, y, theta, x_traj, y_traj,xo_traj, yo_traj,xo,yo):  # pragma: no cover
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T1 = self.transformation_matrix(x, y, theta)
    T2 = self.transformation_matrix(xo, yo, theta)

    p1 = np.matmul(T1, p1_i)
    p2 = np.matmul(T1, p2_i)
    p3 = np.matmul(T1, p3_i)

    p1o = np.matmul(T2, p1_i)
    p2o = np.matmul(T2, p2_i)
    p3o = np.matmul(T2, p3_i)
    
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    plt.plot([p1o[0], p2o[0]], [p1o[1], p2o[1]], 'k-')
    plt.plot([p2o[0], p3o[0]], [p2o[1], p3o[1]], 'k-')
    plt.plot([p3o[0], p1o[0]], [p3o[1], p1o[1]], 'k-')

   
    plt.style.use('seaborn')
    plt.scatter([18,7,10],[15,7,13],s=100,c='red',edgecolor='black',linewidth=2,alpha=0.75)   
    
    plt.plot(x_traj, y_traj, 'b--')
    
    plt.plot(xo_traj, yo_traj,'r--')
    
    
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
    
    plt.xlim(0, 30)
    plt.ylim(0, 30)
    
    plt.pause(self.dt)
  def render(self,mode="human"):
        

        plt.cla()
        plt.arrow(self.x_start, self.y_start, np.cos(self.theta_start),
                      np.sin(self.theta_start), color='r', width=0.1)
        plt.arrow(self.x_goal, self.y_goal, np.cos(self.theta_goal),
                      np.sin(self.theta_goal), color='g', width=0.1)
        self.plot_vehicle(self.pA[0], self.pA[1], self.theta, self.x_traj, self.y_traj,self.xo_traj,self.yo_traj,self.pO[0],self.pO[1])
  def seed():
      return None 
  def metadata(x):
      return 0 
  def legal_actions(self):
    return gym.spaces.Discrete(5)
  def close(self):
    pass

env=modf_football(Football,gym.Env)

from stable_baselines.common.env_checker import check_env

check_env(env, warn=True)

from stable_baselines import DQN, PPO2, A2C, ACKTR
from stable_baselines.common.cmd_util import make_vec_env

env = make_vec_env(lambda: env, n_envs=1)

model = PPO2('MlpPolicy', env, verbose=1).learn(50000)

import matplotlib.pyplot as plt
obs = env.reset()
n_steps = 2000
for step in range(n_steps):
  action, _ = model.predict(obs, deterministic=True)
  print("Step {}".format(step + 1))
  print("Action: ", action)
  obs, reward, done, info = env.step(action)
  print('obs=', obs, 'reward=', reward, 'done=', done)
  env.render(mode='console')
  if done:
    # Note that the VecEnv resets automatically
    # when a done signal is encountered
    print("Goal reached!", "reward=", reward)
    break
env.close()