import pybullet as p
import time
import pybullet_data
import random
import asyncio
import asyncio_test



async def main():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")

    # set POV
    cameraDistance = 1
    cameraPitch = -80
    cameraYaw = 90
    cameraTargetPosition = [0, 0, 0]
    p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

    startPos = [0, 0, 0]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    pong2 = p.loadURDF("pong2.urdf", startPos, startOrientation)
    fixed_pos = [0, 0, 0]  # Change this to the desired fixed position
    p.createConstraint(pong2, -1, -1, -1, p.JOINT_FIXED, fixed_pos, [0, 0, 0], [0, 0, 0])
    for i in range(4):
        p.changeDynamics(pong2, i, restitution=0.5)

    for i in range(100000):
        # Your existing simulation logic
        # ...

        if i % 10 == 0:
            # Execute "shoot and reload" asynchronously
            asyncio.create_task(shoot_and_reload())

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

            spawnpos = random.uniform(-0.2, 0.2)
            startPos = [spawnpos, 0.25, 0.085]
            startOrientation1 = p.getQuaternionFromEuler([0, 0, 0])
            ball = p.loadURDF("ball.urdf", startPos, startOrientation1)
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
        await asyncio.sleep(1./240.)  # Replace with actual non-blocking delay

    # Get final position and orientation
    cubePos, cubeOrn = p.getBasePositionAndOrientation(ball)
    print("Final Position:", cubePos)
    print("Final Orientation:", cubeOrn)

    # Disconnect from PyBullet
    p.disconnect()

async def shoot_and_reload():
    shootVel = 2
    reloadVel = -1
    maxForce = 100000

    # Shooting
    p.setJointMotorControl2(pong2, 3, p.VELOCITY_CONTROL, targetVelocity=shootVel)
    await asyncio.sleep(0.3)  # Replace with actual non-blocking delay

    # Reloading
    p.setJointMotorControl2(pong2, 3, p.VELOCITY_CONTROL, targetVelocity=reloadVel)
    await asyncio.sleep(0.5)  # Replace with actual non-blocking delay
    p.setJointMotorControl2(pong2, 3, p.VELOCITY_CONTROL, targetVelocity=0)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
