from old_model import Pong2Env
from gymnasium import spaces
from gymnasium import Env
import pybullet as p
import pybullet_data
import numpy as np
import time
import threading
import random
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

class Pong2newEnv(Env):
    """Custom environment using OpenAI Gym and PyBullet."""

    def __init__(self):
        # Action space: discrete (left, right, strike, do nothing)
        self.action_space = spaces.Discrete(4)

        # Observation space: continuous (ball x, y, vx, vy, old_striker x, old_striker_vx, new_striker x, new_striker_vx)
        low_limit = np.array([-0.2, -0.5, -3.0, -3.0, -0.1255, -0.55, -0.1255, -0.55])  # Min values
        high_limit = np.array([0.2, 0.5, 3.0, 3.0, 0.1255, 0.55, 0.1255, 0.55])  # Max values
        self.observation_space = spaces.Box(low=low_limit, high=high_limit, shape=(8,))

        # Initialize PyBullet simulation
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

    def reset(self, seed=None):
        # Reset the simulation to the initial state
        p.disconnect(self.client)
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        # Load the plane URDF
        self.planeId = p.loadURDF("plane.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

        # TODO spawn new pong2 urdf
        # Respawn pong2
        startPosPong2 = [0, 0, 0]
        startOrientationPong2 = p.getQuaternionFromEuler([0, 0, 0])
        self.pong2 = p.loadURDF("../URDF/pong2.urdf", startPosPong2, startOrientationPong2)
        p.createConstraint(self.pong2, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
        for i in range(4):
            p.changeDynamics(self.pong2, i, restitution=0.5)

        # Respawn ball
        spawnpos = random.uniform(-0.2, 0.2)
        startPosBall = [spawnpos, 0.25, 0.085]
        startOrientationBall = p.getQuaternionFromEuler([0, 0, 0])
        self.ball = p.loadURDF("../URDF/ball.urdf", startPosBall, startOrientationBall)
        p.changeDynamics(self.ball, -1, restitution=0.5)

        old_env = Pong2Env()
        old_env = Monitor(old_env)

        model = PPO.load('Reinforcement Learning/Training/Saved Models/PPO_Model_Pong2', env=old_env)

        _o, _ = env.reset()
        obs = #TODO get observation for old model (6 parameters)
        done = False
        action, _states = model.predict(obs)
        # TODO setjointmotorcontrol with action


        # Apply a random force to the ball # TODO change force magnitude
        x = random.uniform(-4, 4)
        y = -abs(random.uniform(2, 4))
        z = 0
        p.applyExternalForce(self.ball, -1, [x, y, z], [0, 0, 0], p.WORLD_FRAME)

        # Reset other variables
        self.game_length = 10000
        self.endflag = 0
        self.strikerflag = 0

        # Set the camera position
        cameraDistance = 1
        cameraPitch = -80
        cameraYaw = 90
        cameraTargetPosition = [0, 0, 0]
        p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

        info = {}
        # Return the initial observation
        return self._get_observation(), info

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

        # Shoot
        if action == 2:
            # Prevent agent from spamming action 3
            shoot_penalty = -10
            # Shoot and reload with Threading
            threading.Thread(target=self.shoot_and_reload).start()


        # reduce game length by 1
        self.game_length -= 1

        p.stepSimulation()

        self.state = self._get_observation()  # Get the current observation

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
                reward = 10
                self.strikerflag = 1

        # Ball touches player sensor (Robot Wins)
        player_contacts = p.getContactPoints(self.ball, self.pong2, linkIndexB=5)
        if player_contacts:
            reward = 50
            self.endflag = 1

        # Ball touches robot sensor (Player Wins)
        robot_contacts = p.getContactPoints(self.ball, self.pong2, linkIndexB=4)
        if robot_contacts:
            reward = -50
            self.endflag = 1

        # Higher reward for higher ball velocity after striking
        if self.strikerflag == 1:
            speed_reward = state[3] * 10
        else:
            speed_reward = 0

        length_penalty = (10000 - self.game_length) * 0.025

        reward = reward + speed_reward - length_penalty

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

# log_path = os.path.join('Training', 'Logs')
#
# model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_path)
# model.learn(total_timesteps=1000000)
#
# PPO_Path = os.path.join('Training', 'Saved Models', 'PPO_Model_Pong2')
# model.save(PPO_Path)

# mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
# env=Pong2Env()
# print(env.observation_space.sample())
# print(env.action_space.sample())
#check_env(env, warn=True)
