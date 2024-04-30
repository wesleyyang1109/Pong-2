import gym
from gym import spaces
import pybullet as p
import pybullet_data
import time
import os
import random


class CustomEnv(gym.Env):
    """Custom environment using OpenAI Gym and PyBullet."""

    def __init__(self):
        # Action space: discrete (left, right, strike)
        self.action_space = spaces.Discrete(3)

        # Observation space: continuous (puck x, y, vx, vy, striker x, y, striker_vx)
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
        return self._get_observation()  # Return the initial observation

    def step(self, action):
        # Take an action in the environment
        # Perform actions based on action space (e.g., control robot joints)
        p.stepSimulation()
        reward = self._calculate_reward(action)  # Calculate reward based on action and state
        done = self._is_done()  # Determine if episode is finished
        observation = self._get_observation()  # Get the current observation
        info = {}  # Optional info dictionary
        return observation, reward, done, info

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
        joint_positions, joint_velocities, object_positions = self._get_state_info()
        # Combine or process information into a suitable observation array
        return observation

    def _calculate_reward(self, action):
        # Calculate reward based on the action and current state
        # Example: positive reward for achieving goal, penalty for collisions
        return reward

    def _is_done(self):
        # Check if the episode should terminate
        # Example: robot falls, goal achieved, timeout reached
        return done

    def _get_state_info(self):
        # Helper function to retrieve specific state information from PyBullet
        # Example: joint states, object poses
        joint_positions, joint_velocities, object_positions = [], [], []
        # ... Implement retrieval logic using PyBullet functions (p.getJointState, p.getBasePositionAndOrientation, etc.)
        return joint_positions, joint_velocities, object_positions

