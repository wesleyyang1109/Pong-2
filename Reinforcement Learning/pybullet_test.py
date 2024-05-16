import pybullet as p
import time
import pybullet_data
import os
import random
import time
import threading

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")


#set POV
cameraDistance = 1
cameraPitch = -80
cameraYaw = 90
cameraTargetPosition = [0, 0, 0]
p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)



startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
pong2 = p.loadURDF("URDF/pong2.urdf", startPos, startOrientation)
fixed_pos = [0, 0, 0]  # Change this to the desired fixed position
p.createConstraint(pong2, -1, -1, -1, p.JOINT_FIXED, fixed_pos, [0, 0, 0], [0, 0, 0])
for i in range(4):
    p.changeDynamics(pong2, i, restitution = 0.5)



startPos = [0, 0.25, 0.085]
startOrientation1 = p.getQuaternionFromEuler([0,0,0])
ball = p.loadURDF("URDF/ball.urdf", startPos, startOrientation1)

p.changeDynamics(ball, -1, restitution = 0.5)


# Generate random x and z values between -1 and 1
x = random.uniform(-10, 10)
z = 0
# Ensure negative y-component
y = -abs(random.uniform(0.5, 10))  # Generate negative random value between 0 and 1

p.applyExternalForce(ball, -1, [x, y, z], [0, 0, 0], 1)

def shoot_and_reload():
    shootVel = 2
    p.setJointMotorControl2(pong2, 3, p.VELOCITY_CONTROL, targetVelocity=shootVel)
    time.sleep(0.5)
    # striker = p.getLinkState(pong2, 3, computeLinkVelocity=1)
    # striker_vel = striker[6][0]
    # print("STRIKER VEL FUUUUUCCCKK")
    # print(striker_vel)
    # print("")
    # time.sleep(0.2)

    reloadVel = -0.007
    # p.setJointMotorControl2(pong2, 3, p.POSITION_CONTROL, targetPos, force = maxForce, maxVelocity = maxVel)
    p.setJointMotorControl2(pong2, 3, p.VELOCITY_CONTROL, targetVelocity=reloadVel)
    time.sleep(0.5)
    # striker = p.getLinkState(pong2, 3, computeLinkVelocity=1)
    # striker_vel = striker[6][0]
    # print("STRIKER END")
    # print(striker_vel)
    # print("")
    # time.sleep(0.4)
    p.setJointMotorControl2(pong2, 3, p.VELOCITY_CONTROL, targetVelocity=0)


for i in range (100000):

    #Detection for Rewards
    striker_contacts = p.getContactPoints(ball, pong2, linkIndexB=3)
    if striker_contacts:
        ok = 1
        #ADD REWARD
        #print("Striker - ball")

    player_contacts = p.getContactPoints(ball, pong2, linkIndexB=5)
    if player_contacts:
        ok = 1
        #MINUS REWARD
        #print("Player - ball")

    robot_contacts = p.getContactPoints(ball, pong2, linkIndexB=4)
    if robot_contacts:
        ok = 1
        #ADD MOST REWARD
        #print("Robot - ball")




    #OBSERVATION SPACE (STATES)
    #pong2 link position and vel
    pos = p.getLinkState(pong2, 2, computeLinkVelocity = 1)
    striker_pos_x = pos[0][0]
    striker_pos_y = pos[0][1]
    striker_vel_x = pos[6][0]
    # print(pos)
    # print(striker_pos_x, striker_vel_x)


    #ball position and vel
    ballPos, cubeOrn = p.getBasePositionAndOrientation(ball)
    ball_pos_x = ballPos[0]
    ball_pos_y = ballPos[1]
    ballVel, angvel = p.getBaseVelocity(ball)
    ball_vel_x = ballVel[0]
    ball_vel_y = ballVel[1]
    # print(ball_pos_x, ball_pos_y, ball_vel_x, ball_vel_y)



    if i % 300 == 0:
        targetPos = 0.015
        maxVel = 2
        maxForce = 100000
        # p.setJointMotorControl2(pong2, 3, p.VELOCITY_CONTROL, targetVelocity = maxVel)

        threading.Thread(target=shoot_and_reload).start()



    # if i % 10 == 0:
    #     targetPos = 0.015
    #     maxVel = -1
    #     maxForce = 100000
    #     # p.setJointMotorControl2(pong2, 3, p.POSITION_CONTROL, targetPos, force = maxForce, maxVelocity = maxVel)
    #     p.setJointMotorControl2(pong2, 3, p.VELOCITY_CONTROL, targetVelocity = maxVel)

    if i % 50 == 0:
         print("vel_x, vel_y")
         print(ball_vel_x, ball_vel_y)
         print("")

    if i % 100 == 0:
        # MOVE LEFT
        maxVel = 0.5
        maxForce = 50
        p.setJointMotorControl2(pong2, 2, p.VELOCITY_CONTROL, targetVelocity=maxVel, force=maxForce)

    if i % 200 == 0:
        # MOVE RIGHT
        maxVel = -0.5
        maxForce = 50
        p.setJointMotorControl2(pong2, 2, p.VELOCITY_CONTROL, targetVelocity=maxVel, force=maxForce)


        # targetPos = 0.015
        # maxVel = -1
        # maxForce = 100000
        # # p.setJointMotorControl2(pong2, 3, p.POSITION_CONTROL, targetPos, force = maxForce, maxVelocity = maxVel)
        # p.setJointMotorControl2(pong2, 3, p.VELOCITY_CONTROL, targetVelocity=maxVel)

        spawnpos = random.uniform(-0.2, 0.2)
        startPos = [spawnpos, 0.25, 0.085]
        startOrientation1 = p.getQuaternionFromEuler([0, 0, 0])
        ball = p.loadURDF("URDF/ball.urdf", startPos, startOrientation1)
        p.changeDynamics(ball, -1, restitution=0.5)

        # Generate random x and z values between -1 and 1
        x = random.uniform(-4, 4)
        z = 0
        # Ensure negative y-component (only towards robot)
        y = -abs(random.uniform(2, 4))  # Generate negative random value between 1 and 6
        # x = 0
        # y = -6
        # z = 0

        p.applyExternalForce(ball, -1, [x, y, z], [0, 0, 0], 1)

    p.stepSimulation()
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(ball)
print(cubePos, cubeOrn)
p.disconnect()