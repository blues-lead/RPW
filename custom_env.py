# -*- coding: utf-8 -*-
"""
Created on Thu Jan  2 13:00:55 2020

@author: Anton
stable_baselines supports tensorflow v.1.8.0 - 1.14.0
for now as observation distance to target will be used:
    value=client.getPosition()
    value.distance_to(TARGET)
"""

import gym
import numpy as np
from gym import spaces
import airsim
import time
from stable_baselines.common.env_checker import check_env
from stable_baselines.common.policies import MlpPolicy
from stable_baselines import PPO1
from stable_baselines import TRPO
from stable_baselines.common.vec_env import DummyVecEnv

TARGET_POSITION = airsim.Vector3r(-37.8, -101.7, 0.23)

#DISCRETE_ACTIONS = {
#    0: [0.0, 0.0],    # Coast
#    1: [0.0, -0.5],   # Turn Left
#    2: [0.0, 0.5],    # Turn Right
#    3: [1.0, 0.0],    # Forward
#    4: [-0.5, 0.0],   # Brake
#    5: [1.0, -0.5],   # Bear Left & accelerate
#    6: [1.0, 0.5],    # Bear Right & accelerate
#    7: [-0.5, -0.5],  # Bear Left & decelerate
#    8: [-0.5, 0.5],   # Bear Right & decelerate
#}

class UEEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self):
        super(UEEnv, self).__init__()
        #self.action_space = spaces.Discrete(len(DISCRETE_ACTIONS))
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self._spec_id = "UEMaze"
        self._seed = 1
        self.observation_space = spaces.Box(0,500.0,shape=(1,))
        self.total_reward = 0
        self.num_steps = 0
        self.my_pos = None
        self.end_pos = None
        self.start_coord = None
        self.end_coord = TARGET_POSITION
        self.last_obs = None
        self.prev_measurement = None
        self.reward_range = (-float('inf'), float('inf'))
        self.target_reached = False
        self.prev_speed = 0.0
        
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()
        
    def reset(self):
        self.num_steps = 0
        self.total_reward = 0
        self.prev_measurement = None
        self.my_pos = airsim.Vector3r([0,0,0])
        self.end_pos = TARGET_POSITION
        self.client.reset()
        time.sleep(3)
        self.prev_measurement = airsim.Vector3r(0,0,0)
        return np.array([TARGET_POSITION.distance_to(airsim.Vector3r(0,0,0))])
        
    def step(self,action):
        reward = 0.0
        self.car_controls.throttle = float(np.clip(action[0],0,1))
        self.car_controls.brake = float(np.abs(np.clip(action[0], -1, 0)))#
        self.car_controls.steering = float(np.clip(action[1],-1,1))#
        self.car_controls.handbrake = False#
        self.client.setCarControls(self.car_controls)
        time.sleep(0.01)
        collision = self.client.simGetCollisionInfo()
        
        steer_angle = float(np.clip(action[0],-1,1))
        pos = self.client.simGetGroundTruthKinematics()
        pos = pos.position
        speed = self.client.getCarState()
        speed = speed.speed
        d_dist=pos.distance_to(TARGET_POSITION)-self.prev_measurement.distance_to(TARGET_POSITION)
        
#        reward += 0.5*(speed - self.prev_speed)
        reward += (speed - self.prev_speed)
        reward += np.clip(pos.distance_to(self.prev_measurement),-10,10)
        reward += -5*d_dist
        #reward += abs(steer_angle)*(-0.5)
        
        self.prev_measurement = pos
        self.prev_speed = speed
        if pos.distance_to(TARGET_POSITION) < 5:
            self.target_reached = True
        #self.total_reward += 1/pos.distance_to(TARGET_POSITION)
        done = False
        if collision.has_collided:
            reward = -1000.0
            done = True
            self.client.reset()
            return np.array([pos.distance_to(TARGET_POSITION)]), reward, done, {}
            
        if self.target_reached:
            reward += 1000.0
            done = True
            return np.array([pos.distance_to(TARGET_POSITION)]), reward+100, done, {}
        
        if self.total_reward < -50 or self.total_reward > 2000:
            done = True
            return np.array([pos.distance_to(TARGET_POSITION)]), 0, done, {}
        
        self.total_reward += reward    
        print("reward:", reward, "distance:", pos.distance_to(TARGET_POSITION), "total reward:", self.total_reward)
        return np.array([pos.distance_to(TARGET_POSITION)]), reward, done, 0
            
        
        
agent = DummyVecEnv([lambda: UEEnv()])
#check_env(agent,warn=True,skip_render_check=True)
#obs = agent.reset()
modelP = PPO1(MlpPolicy, agent, verbose=1)
modelT = TRPO(MlpPolicy, agent, verbose=1)
modelP.learn(total_timesteps = 50000)
modelP.save("PPO_UEE_model")
obs = agent.reset()
while True:
    action, states = modelT.predict(obs)
    obs, rewards, dones, _ = agent.step(action)