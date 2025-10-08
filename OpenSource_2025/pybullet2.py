import pybullet as p
import pybullet_data
import time
import math

# --- Setup ---
cid = p.connect(p.GUI)
# access URDF default assets
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# set gravity (x, y, z)
p.setGravity(0, 0, -9.8)

# steps 240hz
dt = 1.0 / 240.0
p.setTimeStep(dt)

# Cross-platform ESC handling (Windows builds may not have p.B3G_ESCAPE)
ESC_KEYS = [27]  # ASCII ESC
if hasattr(p, "B3G_ESCAPE"):
    ESC_KEYS.append(p.B3G_ESCAPE)

# load plane from "plane.urdf", returns an obj id
plane = p.loadURDF("plane.urdf")
# -1 = base link, add friction
p.changeDynamics(plane, -1, lateralFriction=1.0)

# define physical shape of the wall for collision
wall_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.0, 0.1, 0.2])
# define appearance of the wall
wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[1.0, 0.1, 0.2], rgbaColor=[1, 0, 0, 1])
wall = p.createMultiBody(
    baseMass=0.0,
    baseCollisionShapeIndex=wall_collision,
    baseVisualShapeIndex=wall_visual,
    basePosition=[1.0, 0.0, 0.2],
)

p.changeDynamics(wall, -1, lateralFriction=1.0)

# --- Create Robot (simple cylinder) ---
radius = 0.2
height = 0.2  # a bit taller = more stable
base_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
# visual cylinder uses "length" not "height" pybullet uses it like that
base_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=[0, 0, 1, 1])

robot = p.createMultiBody(
    baseMass=1.0, # kg
    baseCollisionShapeIndex=base_collision, # attach collision to shape
    baseVisualShapeIndex=base_visual, # attach visual to shape
    basePosition=[0.0, 0.0, height * 0.5], # position of robot (center is half the height)
)
# linear damping: reduces linear velocity
# angular damping: reduces rotational velocity (prevents spinning forever)
# lateral friction: higher = less sliding
p.changeDynamics(robot, -1, linearDamping=0.04, angularDamping=0.04, lateralFriction=1.2)

# --- Control parameters ---
velocity = 1.0     # linear speed (m/s)
turn_speed = 1.5   # yaw rate (rad/s)

print("Use arrows to move the robot. Press Q or ESC to quit.")

try:
    while True:
        keys = p.getKeyboardEvents()

        # Quit if Q or ESC was just pressed
        if (keys.get(ord('q'), 0) & p.KEY_WAS_TRIGGERED) or any(
            keys.get(k, 0) & p.KEY_WAS_TRIGGERED for k in ESC_KEYS
        ):
            break

        linear = 0.0
        angular = 0.0
        if keys.get(p.B3G_UP_ARROW, 0) & p.KEY_IS_DOWN:
            linear = velocity
        if keys.get(p.B3G_DOWN_ARROW, 0) & p.KEY_IS_DOWN:
            linear = -velocity
        if keys.get(p.B3G_LEFT_ARROW, 0) & p.KEY_IS_DOWN:
            angular = +turn_speed
        if keys.get(p.B3G_RIGHT_ARROW, 0) & p.KEY_IS_DOWN:
            angular = -turn_speed

        # Read current yaw to convert body-forward to world-frame velocity
        _, orn = p.getBasePositionAndOrientation(robot)
        _, _, yaw = p.getEulerFromQuaternion(orn)

        vx = linear * math.cos(yaw)
        vy = linear * math.sin(yaw)

        # Apply velocities instead of resetting pose (so collisions work)
        p.resetBaseVelocity(robot, linearVelocity=[vx, vy, 0.0], angularVelocity=[0.0, 0.0, angular])

        p.stepSimulation()
        time.sleep(dt)
finally:
    p.disconnect()
