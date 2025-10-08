import pybullet as p
import pybullet_data
import math
import time
import random

# --- Setup ---
cid = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
p.setGravity(0, 0, -9.8)
dt = 1.0 / 240.0
p.setTimeStep(dt)

# --- Environment: Plane + Obstacles ---
plane = p.loadURDF("plane.urdf")
p.changeDynamics(plane, -1, lateralFriction=1.0)


def random_walls(n_walls):
    # Add random walls
    for _ in range(n_walls):
        x = random.uniform(-2, 2)
        y = random.uniform(-2, 2)
        wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.8, 0.3])
        wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.8, 0.3], rgbaColor=[1, 0, 0, 1])
        p.createMultiBody(0, wall_col, wall_vis, [x, y, 0.3])


def create_maze():
    # Maze layout (1 = wall, 0 = empty)
    maze_layout = [
        [1,1,1,1,1],
        [1,0,0,0,1],
        [1,0,1,0,1],
        [1,0,0,0,1],
        [1,1,1,1,1],
    ]

    wall_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.25])
    wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.25], rgbaColor=[0.8, 0.2, 0.2, 1])

    for i, row in enumerate(maze_layout):
        for j, cell in enumerate(row):
            if cell == 1:
                p.createMultiBody(
                    baseMass=0,
                    baseCollisionShapeIndex=wall_collision,
                    baseVisualShapeIndex=wall_visual,
                    basePosition=[i * 0.5, j * 0.5, 0.25],
                )


# decide which scenario to play
create_maze()

# --- Robot: Simple Cylinder ---
radius = 0.15
height = 0.1
robot_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
robot_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=[0, 0, 1, 1])
# robot position z=0.1 just above the plane
robot = p.createMultiBody(1.0, robot_col, robot_vis, [0.3, 0.3, 0.1])

p.changeDynamics(robot, -1, lateralFriction=1.2)

# --- Movement Settings ---
velocity = 1.0
turn_speed = 2.0

# --- Ray Sensor Settings ---
sensor_length = 1.5
sensor_angles = [-45, 0, 45]  # degrees
auto_mode = True

print("Robot Explorer running — use 'M' to toggle manual/auto mode, 'WASD' to move manually.")

while True:
    keys = p.getKeyboardEvents()

    # Quit
    if ord('q') in keys:
        break

    # Toggle auto/manual
    if ord('m') in keys and keys[ord('m')] & p.KEY_WAS_TRIGGERED:
        auto_mode = not auto_mode
        print("Auto mode:", auto_mode)

    # Get robot state
    pos, orn = p.getBasePositionAndOrientation(robot)
    x, y, _ = pos
    _, _, yaw = p.getEulerFromQuaternion(orn)

    linear = 0
    angular = 0

    # --- Manual Control ---
    if not auto_mode:
        if keys.get(p.B3G_UP_ARROW, 0) & p.KEY_IS_DOWN:
            linear = velocity
        if keys.get(p.B3G_DOWN_ARROW, 0) & p.KEY_IS_DOWN:
            linear = -velocity
        if keys.get(p.B3G_LEFT_ARROW, 0) & p.KEY_IS_DOWN:
            angular = +turn_speed
        if keys.get(p.B3G_RIGHT_ARROW, 0) & p.KEY_IS_DOWN:
            angular = -turn_speed

    # --- Auto Mode (Reactive control) ---
    else:
        hit_distances = []
        for angle_deg in sensor_angles:
            ray_angle = yaw + math.radians(angle_deg)
            from_pos = [x, y, 0.15]
            to_pos = [
                x + sensor_length * math.cos(ray_angle),
                y + sensor_length * math.sin(ray_angle),
                0.15,
            ]
            result = p.rayTest(from_pos, to_pos)[0]
            hit_distances.append(result[2] if result[0] != -1 else sensor_length)

            color = [1, 0, 0] if result[0] != -1 else [0, 1, 0]
            p.addUserDebugLine(from_pos, to_pos, color, 1, lifeTime=dt)

        # Unpack rays: [left, front, right]
        left, front, right = hit_distances

        # --- Simple avoidance logic ---
        if front < 0.6:  # Obstacle ahead
            # Turn toward the side with more free space
            angular = turn_speed if left > right else -turn_speed
        else:
            linear = velocity  # Go straight

    # --- Move Robot ---
    new_x = x + linear * math.cos(yaw) * dt
    new_y = y + linear * math.sin(yaw) * dt
    new_yaw = yaw + angular * dt

    # --- Camera follow ---
    cam_distance = 2.5   # how far the camera is from the robot
    cam_yaw_offset = 180 # rotate camera 180° behind the robot
    cam_pitch = -30      # camera tilt angle

    cam_yaw = math.degrees(yaw) + cam_yaw_offset
    target_pos = [new_x, new_y, 0.2]

    p.resetDebugVisualizerCamera(
        cameraDistance=cam_distance,
        cameraYaw=cam_yaw,
        cameraPitch=cam_pitch,
        cameraTargetPosition=target_pos
    )


    p.resetBasePositionAndOrientation(
        robot,
        [new_x, new_y, 0.1],
        p.getQuaternionFromEuler([0, 0, new_yaw])
    )

    p.stepSimulation()
    time.sleep(dt)
