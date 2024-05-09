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

class Pong2Env(Env):
    """Custom environment using OpenAI Gym and PyBullet."""

    def __init__(self):
        # Action space: discrete (left, right, strike, do nothing)
        self.action_space = spaces.Discrete(4)

        # Observation space: continuous (ball x, y, vx, vy, striker x, striker_vx)
        low_limit = np.array([-0.19, -0.5, -1.0, -3.0, -0.1255, -0.55])  # Min values
        high_limit = np.array([0.19, 0.5, 1.0, 3.0, 0.1255, 0.55])  # Max values
        self.observation_space = spaces.Box(low=low_limit, high=high_limit, shape=(6,))

        # Initialize PyBullet simulation
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

    def reset(self, seed=None):
        # Reset the simulation to the initial state
        p.resetSimulation(self.client)

        # Load the plane URDF
        self.planeId = p.loadURDF("plane.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

        # Respawn pong2
        startPosPong2 = [0, 0, 0]
        startOrientationPong2 = p.getQuaternionFromEuler([0, 0, 0])
        self.pong2 = p.loadURDF("pong2.urdf", startPosPong2, startOrientationPong2)
        p.createConstraint(self.pong2, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
        for i in range(4):
            p.changeDynamics(self.pong2, i, restitution=0.5)

        # Respawn ball
        spawnpos = random.uniform(-0.2, 0.2)
        startPosBall = [spawnpos, 0.25, 0.085]
        startOrientationBall = p.getQuaternionFromEuler([0, 0, 0])
        self.ball = p.loadURDF("ball.urdf", startPosBall, startOrientationBall)
        p.changeDynamics(self.ball, -1, restitution=0.5)

        # Apply a random force to the ball
        x = random.uniform(-4, 4)
        y = -abs(random.uniform(2, 4))
        z = 0
        p.applyExternalForce(self.ball, -1, [x, y, z], [0, 0, 0], p.WORLD_FRAME)

        # Reset other variables
        self.game_length = 500
        self.flag = 0

        # Set the camera position
        cameraDistance = 1
        cameraPitch = -80
        cameraYaw = 90
        cameraTargetPosition = [0, 0, 0]
        p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

        # Return the initial observation
        return self._get_observation()

    def step(self, action):
    # Perform actions based on action space
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

        # Shoot
        if action == 2:
            # Shoot and reload with Threading
            threading.Thread(target=self.shoot_and_reload).start()
            # Prevent agent from spamming action 3
            shoot_penalty = -1

        # reduce game length by 1
        self.game_length -= 1

        p.stepSimulation()

        self.state = self._get_observation()  # Get the current observation

        # Calculate Reward
        reward = shoot_penalty + self._calculate_reward(self.state)  # Calculate reward based on action and state

        done = self._is_done()  # Determine if episode is finished
        info = {}  # Optional info dictionary
        return self.state, reward, done, info

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

        # TODO
        # Combine or process information into a suitable observation array   (ball x, y, vx, vy, striker x, y, striker_vx)
        observation = np.concatenate((ball_pos_x, ball_pos_y, ball_vel_x, ball_vel_y, striker_pos_x, striker_vel_x))
        return observation

    def _calculate_reward(self, state):
    # Calculate reward based on the action and current state

        # Striker touches Ball
        striker_contacts = p.getContactPoints(self.ball, self.pong2, linkIndexB=3)
        if striker_contacts:
            reward = 3

        # Ball touches player sensor (Robot Wins)
        player_contacts = p.getContactPoints(self.ball, self.pong2, linkIndexB=5)
        if player_contacts:
            reward = 5
            self.flag = 1

        # Ball touches robot sensor (Player Wins)
        robot_contacts = p.getContactPoints(self.ball, self.pong2, linkIndexB=4)
        if robot_contacts:
            reward = -4
            self.flag = 1

        return reward

    def _is_done(self):
        # Check if the game length has reached 0
        if self.game_length <= 0:
            done = True
        # Check if any side scores
        elif self.flag == 1:
            done = True
        else:
            done = False
        return done

    def shoot_and_reload(self):
        maxVel = 2
        #maxForce = 100000
        p.setJointMotorControl2(self.pong2, 3, p.VELOCITY_CONTROL, targetVelocity=maxVel)
        time.sleep(0.5)

        maxVel = -1
        # p.setJointMotorControl2(pong2, 3, p.POSITION_CONTROL, targetPos, force = maxForce, maxVelocity = maxVel)
        p.setJointMotorControl2(self.pong2, 3, p.VELOCITY_CONTROL, targetVelocity=maxVel)
        time.sleep(0.5)
        p.setJointMotorControl2(self.pong2, 3, p.VELOCITY_CONTROL, targetVelocity=0)


env=Pong2Env()
print(env.observation_space.sample())
print(env.action_space.sample())
check_env(env, warn=True)
