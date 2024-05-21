import gymnasium as gym
from gymnasium import spaces
from gymnasium import Env
import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import random
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecFrameStack
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor

class Pong2Env(Env):
    """Custom environment using OpenAI Gym and PyBullet."""

    def __init__(self):
        # Action space: discrete (left, right, strike, do nothing)
        self.action_space = spaces.Discrete(4)

        # Observation space: continuous (ball x, y, vx, vy, striker x, striker_vx)
        low_limit = np.array([-0.2, -0.5, -3.0, -3.0, -0.09, -0.55])  # Min values
        high_limit = np.array([0.2, 0.5, 3.0, 3.0, 0.09, 0.55])  # Max values
        self.observation_space = spaces.Box(low=low_limit, high=high_limit, shape=(6,))

        # Initialize PyBullet simulation
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

    def reset(self, seed=None):
        # Reset the simulation to the initial state
        p.disconnect(self.client)
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        # Load the plane URDF
        self.planeId = p.loadURDF("plane.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

        # Respawn pong2
        startPosPong2 = [0, 0, 0]
        startOrientationPong2 = p.getQuaternionFromEuler([0, 0, 0])
        self.pong2 = p.loadURDF("../URDF/pong2.urdf", startPosPong2, startOrientationPong2)
        p.createConstraint(self.pong2, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
        for i in range(4):
            p.changeDynamics(self.pong2, i, restitution=0.5)

        # Respawn ball
        spawnpos = random.uniform(-0.2, 0.2)
        startPosBall = [spawnpos, 0.22, 0.085]
        startOrientationBall = p.getQuaternionFromEuler([0, 0, 0])
        self.ball = p.loadURDF("../URDF/ball.urdf", startPosBall, startOrientationBall)
        p.changeDynamics(self.ball, -1, restitution=0.5)

        # TODO change force magnitude
        # Apply a random force to the ball
        x = random.uniform(-2, 2)
        y = -abs(random.uniform(1, 2))
        z = 0
        p.applyExternalForce(self.ball, -1, [x, y, z], [0, 0, 0], p.WORLD_FRAME)

        # Reset other variables
        self.game_length = 10000
        self.endflag = 0
        self.strikerflag = 0
        self.shootflag = 0

        # Set the camera position
        cameraDistance = 1
        cameraPitch = -80
        cameraYaw = 90
        cameraTargetPosition = [0, 0, 0]
        p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

        info = {}
        # Return the initial observation
        return self._get_observation(), info

    # TODO fix shoot and reload
    def step(self, action):
    # Perform actions based on action space
        shoot_penalty = 0
        # Left
        if action == 0:
            maxVel = 0.5
            maxForce = 50
            p.setJointMotorControl2(self.pong2, 2, p.VELOCITY_CONTROL, targetVelocity=maxVel, force=maxForce)

        # Right
        if action == 1:
            maxVel = -0.5
            maxForce = 50
            p.setJointMotorControl2(self.pong2, 2, p.VELOCITY_CONTROL, targetVelocity=maxVel, force=maxForce)

        # Check if agent has shoot
        if self.shootflag == 0:
            p.setJointMotorControl2(self.pong2, 3, p.VELOCITY_CONTROL, targetVelocity=0)
            # Shoot
            if action == 2:
                # Prevent agent from spamming action 3
                ## shoot_penalty = -0.5
                # shoot
                maxVel = 1
                p.setJointMotorControl2(self.pong2, 3, p.VELOCITY_CONTROL, targetVelocity=maxVel)

                self.shootflag = 340

        elif self.shootflag == 250:
            maxVel = -0.025
            p.setJointMotorControl2(self.pong2, 3, p.VELOCITY_CONTROL, targetVelocity=maxVel)
            self.shootflag -= 1
        else:
            self.shootflag -= 1


        # reduce game length by 1
        self.game_length -= 1

        p.stepSimulation()

        self.state = self._get_observation()  # Get the current observation

        shoot_penalty = 0
        # Calculate Reward
        reward = shoot_penalty + self._calculate_reward(self.state)  # Calculate reward based on action and state

        done = self._is_done()  # Determine if episode is finished
        truncated = False
        info = {}  # Optional info dictionary
        return self.state, reward, done, truncated, info

    def render(self, mode='human'):
        # Render the environment
        pass

    def close(self):
        # Close the PyBullet simulation
        p.disconnect(self.client)

    def _get_observation(self):
    # Gather relevant information from the simulation for observation
    # Example: joint positions, velocities, object positions

        # pong2 link position and vel
        pos = p.getLinkState(self.pong2, 2, computeLinkVelocity=1)
        striker_pos_x = pos[0][0]
        striker_vel_x = pos[6][0]

        # ball position and vel
        ballPos, cubeOrn = p.getBasePositionAndOrientation(self.ball)
        ball_pos_x = ballPos[0]
        ball_pos_y = ballPos[1]
        ballVel, angvel = p.getBaseVelocity(self.ball)
        ball_vel_x = ballVel[0]
        ball_vel_y = ballVel[1]

        # Combine or process information into a suitable observation array   (ball x, y, vx, vy, striker x, striker_vx)
        observation = np.concatenate((np.array([ball_pos_x]), np.array([ball_pos_y]), np.array([ball_vel_x]), np.array([ball_vel_y]), np.array([striker_pos_x]), np.array([striker_vel_x])))
        observation = observation.astype(np.float32)
        return observation

    def _calculate_reward(self, state):
    # Calculate reward based on the action and current state

        reward = 0
        # If striker hasn't touch the ball at all
        if self.strikerflag == 0:
            # Striker touches Ball
            striker_contacts = p.getContactPoints(self.ball, self.pong2, linkIndexB=3)
            if striker_contacts:
                reward = 5
                self.strikerflag = 1

        # Ball touches player sensor (Robot Wins)
        player_contacts = p.getContactPoints(self.ball, self.pong2, linkIndexB=5)
        if player_contacts:
            reward = 10
            self.endflag = 1

        # Ball touches robot sensor (Player Wins)
        robot_contacts = p.getContactPoints(self.ball, self.pong2, linkIndexB=4)
        if robot_contacts:
            reward = -10
            self.endflag = 1

        # TODO maybe add it gets triggered only when action 2 is picked
        # Ball speed reward only gets triggered when close to striker and is moving away from it
        if state[1] == -0.15 and state[3] >= 1:
            # Higher reward for higher ball velocity after striking
            speed_reward = state[3] * 10
        else:
            speed_reward = 0

        #length_penalty = (10000 - self.game_length) * 0.025

        reward = reward + speed_reward# - length_penalty

        return reward

    def _is_done(self):
        # Check if the game length has reached 0
        if self.game_length <= 0:
            done = True
        # Check if any side scores
        elif self.endflag == 1:
            done = True
        else:
            done = False
        return done


# env = Pong2Env()
# env = Monitor(env)
# check_env(env, warn=True)


# Test Environment
# episodes = 1
# for episode in range(1, episodes + 1):
#     state = env.reset()
#     done = False
#     score = 0
#
#     while not done:
#         env.render()
#         action = env.action_space.sample()
#         n_state, reward, done, _, info = env.step(action)
#         score += reward
#     print('Episode:{} Score:{}'.format(episode, score))
# env.close()



# mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
# env=Pong2Env()
# print(env.observation_space.sample())
# print(env.action_space.sample())
#check_env(env, warn=True)
