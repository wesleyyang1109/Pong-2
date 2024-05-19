from old_model import Pong2Env
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

env = Pong2Env()
env = Monitor(env)

log_path = os.path.join('Training', 'Logs')

model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_path)
model.learn(total_timesteps=1000000, progress_bar=True)

PPO_Path = os.path.join('Training', 'Saved Models', 'PPO_Model_Pong2')
model.save(PPO_Path)