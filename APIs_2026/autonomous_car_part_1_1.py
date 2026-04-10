import pybullet as p
import time
import pybullet_data

# load the physics server 
physicsClient = p.connect(p.GUI)

# use additional robots + planes (show what happens if we miss this)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# set the gravity
p.setGravity(0, 0, -9.81)

# laod the plane
planeId = p.loadURDF("plane.urdf")

# define starting position (explain why the 1 in z)
startPos = [0, 0, 1]

# explain a bit what a quaternion is and what Euler's angles are
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

# load the robot as box
boxId = p.loadURDF("r2d2.urdf", startPos, startOrientation)

# use a for loop to control the system
for i in range (10000):
    # have the physics server step
    p.stepSimulation()
    # if pybullet runs to 240, we slow it down to 1
    time.sleep(1./240.)
    
# get the position and orientation
cube_pos, cube_orn = p.getBasePositionAndOrientation(boxId)
print(cube_pos,cube_orn)
p.disconnect()
