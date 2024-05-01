import gym
from gym import spaces
import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import random


class CustomEnv(gym.Env):
    """Custom environment using OpenAI Gym and PyBullet."""

    def __init__(self):
        # Action space: discrete (left, right, strike)
        self.action_space = spaces.Discrete(3)

        # Observation space: continuous (ball x, y, vx, vy, striker x, y, striker_vx)
        low_limit = np.array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0])  # Min values
        high_limit = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])  # Max values
        self.observation_space = spaces.Box(low=low_limit, high=high_limit, shape=(7,))

        # Initialize PyBullet simulation
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        # Set up your environment elements here (robots, objects, etc.)
        self.planeId = p.loadURDF("plane.urdf")  # Example: Load a plane

        # Load pong2 urdf
        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        pong2 = p.loadURDF("pong2.urdf", startPos, startOrientation)
        # Fix pong2 to plane
        fixed_pos = [0, 0, 0]
        p.createConstraint(pong2, -1, -1, -1, p.JOINT_FIXED, fixed_pos, [0, 0, 0], [0, 0, 0])
        # Change bounciness of pong2
        for i in range(4):
            p.changeDynamics(pong2, i, restitution=0.5)

        # Load ball urdf
        spawnpos = random.uniform(-0.2, 0.2)
        startPos = [spawnpos, 0.25, 0.085]
        startOrientation1 = p.getQuaternionFromEuler([0, 0, 0])
        ball = p.loadURDF("ball.urdf", startPos, startOrientation1)
        # Change bounciness of ball
        p.changeDynamics(ball, -1, restitution=0.5)

        # Apply force randomly on ball
        x = random.uniform(-4, 4)  # Generate random x values between -4 and 4
        z = 0
        y = -abs(random.uniform(2, 4))  # Ensure negative y-component (only towards robot)
        p.applyExternalForce(ball, -1, [x, y, z], [0, 0, 0], 1)

        # Set GUI POV
        cameraDistance = 1
        cameraPitch = -80
        cameraYaw = 90
        cameraTargetPosition = [0, 0, 0]
        p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

    def reset(self):
        # Reset the simulation and environment state
        p.resetSimulation()
        # Set initial positions or states of objects here
        # TODO initial states
        return self._get_observation()  # Return the initial observation

    def step(self, action):
    # Perform actions based on action space
        # Left
        if action == 0:
            maxVel = 0.5
            maxForce = 50
            p.setJointMotorControl2(pong2, 2, p.VELOCITY_CONTROL, targetVelocity=maxVel, force=maxForce)

        # Right
        if action == 1:
            maxVel = -0.5
            maxForce = 50
            p.setJointMotorControl2(pong2, 2, p.VELOCITY_CONTROL, targetVelocity=maxVel, force=maxForce)

        # Shoot
        if action == 2:
            maxVel = 2
            maxForce = 100000
            p.setJointMotorControl2(pong2, 3, p.VELOCITY_CONTROL, targetVelocity=maxVel)
            #TODO Reload with Threading

        p.stepSimulation()

        self.state = self._get_observation()  # Get the current observation

        #TODO Calculate Reward
        reward = self._calculate_reward(self.state)  # Calculate reward based on action and state

        done = self._is_done()  # Determine if episode is finished
        info = {}  # Optional info dictionary
        return self.state, reward, done, info

    def render(self, mode='human'):
        # Render the environment (optional)
        # Use PyBullet's rendering functions or custom visualization tools
        return None  # Or return an image of the environment for 'human' mode

    def close(self):
        # Close the PyBullet simulation
        p.disconnect(self.client)

    def _get_observation(self):
    # Gather relevant information from the simulation for observation
    # Example: joint positions, velocities, object positions

        # pong2 link position and vel
        pos = p.getLinkState(pong2, 2, computeLinkVelocity=1)
        striker_pos_x = pos[0][0]
        striker_pos_y = pos[0][1]
        striker_vel_x = pos[6][0]

        # ball position and vel
        ballPos, cubeOrn = p.getBasePositionAndOrientation(ball)
        ball_pos_x = ballPos[0]
        ball_pos_y = ballPos[1]
        ballVel, angvel = p.getBaseVelocity(ball)
        ball_vel_x = ballVel[0]
        ball_vel_y = ballVel[1]

        # Combine or process information into a suitable observation array   (ball x, y, vx, vy, striker x, y, striker_vx)
        observation = np.concatenate((ball_pos_x, ball_pos_y, ball_vel_x, ball_vel_y, striker_pos_x, striker_pos_y, striker_vel_x))
        return observation

    def _calculate_reward(self, state):
    # Calculate reward based on the action and current state
    # Example: positive reward for achieving goal, penalty for collisions
        #Detection for Rewards
        striker_contacts = p.getContactPoints(ball, pong2, linkIndexB=3)
        if striker_contacts:
            #ADD REWARD
            print("Striker - ball")
        player_contacts = p.getContactPoints(ball, pong2, linkIndexB=5)
        if player_contacts:
            #MINUS REWARD
            print("Player - ball")
        robot_contacts = p.getContactPoints(ball, pong2, linkIndexB=4)
        if robot_contacts:
            #ADD MOST REWARD
            print("Robot - ball")
        return reward

    def _is_done(self):
        # Check if the episode should terminate
        # Example: robot falls, goal achieved, timeout reached
        return done

