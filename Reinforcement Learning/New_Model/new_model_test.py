from new_model import Pong2newEnv
import gymnasium as gym
from gymnasium import spaces
from gymnasium import Env
import pybullet as p
import pybullet_data
import numpy as np
import time
import threading
import os
import random
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecFrameStack
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor


env = Pong2newEnv()
env = Monitor(env)

model = PPO.load('Training/Saved Models/PPO_Model_Pong2', env=env)
# episode = 1
# score = 0
# obs, _ = env.reset()
# print(obs)
# obs = np.array([-0.2, -0.5, 3.0, 3.0, 0.1255, -0.55, 0.1255, -0.55])
# action, _states = model.predict(obs)
# print(action)

episodes = 1
for episode in range(1, episodes+1):
    obs, _ = env.reset()
    done = False
    score = 0

    while not done:
        # env.render()
        action, _states = model.predict(obs)
        obs, reward, done, info, ok = env.step(action)
        score += reward
    print('Episode:{} Score:{}'.format(episode, score))
env.close()