import pybullet as p
import time
import pybullet_data
import math


class defineSimulation:

    def __init__(self, track_points=[]):
        # load the physics server 
        self.physics_client = p.connect(p.GUI)
        # use additional robots + planes (show what happens if we miss this)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # set the gravity
        p.setGravity(0, 0, -9.81)
        # define starting position (explain why the 1 in z)
        self.start_pos = [0, 0, 1]
        # explain a bit what a quaternion is and what Euler's angles are
        self.start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        # load the plane
        self.plane_id = p.loadURDF("plane.urdf")
        # get track points to draw
        self.track_points = track_points
        # to avoid recreating the same walls over
        self.created = False

    def spawn_robot(self, urdf):
        """
        Function that spawns a robot in the plane.
        """
        # load the robot as box
        self.box_id = p.loadURDF(urdf, self.start_pos, self.start_orientation)


    def place_sensors(self, sensor="lidar"):
        """
        Function that 'activates' the sensor of a robot (do this after showing some robots)
        Sensors to be used are:
        - Lidar
        - Position Orientation
        - Camera
        """
        # get the position and orientation (we need to see where to mount it)
        pos, orn = p.getBasePositionAndOrientation(self.box_id)
        # get the rotation matrix (explained before)
        rot_matrix = p.getMatrixFromQuaternion(orn)
        if sensor == "camera":
            forward = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
            camera_eye = [pos[0], pos[1], pos[2] + 0.3]
            camera_target = [
                pos[0] + forward[0],
                pos[1] + forward[1],
                pos[2] + 0.2
            ]

            viewMatrix = p.computeViewMatrix(
                cameraEyePosition=camera_eye,
                cameraTargetPosition=camera_target,
                cameraUpVector=[0, 0, 1]
            )
            projectionMatrix = p.computeProjectionMatrixFOV(
                fov=60,
                aspect=320 / 240,
                nearVal=0.1,
                farVal=100
            )
            #! this makes the simulation go slow! explain why (heavy load)
            _, _, _, _, _ = p.getCameraImage(
                width=320,
                height=240,
                viewMatrix=viewMatrix,
                projectionMatrix=projectionMatrix
            )

        elif sensor == "lidar":
            forward = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
            # Normalize forward
            norm = math.sqrt(forward[0]**2 + forward[1]**2 + forward[2]**2)
            forward = [f / norm for f in forward]
            # configure the lidar
            num_rays = 15
            ray_length = 10
            spread_angle = math.pi / 2  # 90 degrees
            ray_from = []
            ray_to = []

            # generate the "rotating" shape
            for i in range(num_rays):
                angle = -spread_angle/2 + (spread_angle * i / (num_rays - 1))
                # rotate forward vector in XY plane
                dx = forward[0] * math.cos(angle) - forward[1] * math.sin(angle)
                dy = forward[0] * math.sin(angle) + forward[1] * math.cos(angle)

                start = [pos[0], pos[1], pos[2] + 0.2]
                end = [
                    pos[0] + dx * ray_length,
                    pos[1] + dy * ray_length,
                    pos[2] + 0.2
                ]
                ray_from.append(start)
                ray_to.append(end)

            # cast the rays
            results = p.rayTestBatch(ray_from, ray_to)
            # store distances
            self.distances = []

            for i, r in enumerate(results):
                hit_fraction = r[2]  # 0 = hit immediately, 1 = no hit

                dist = hit_fraction * ray_length
                self.distances.append(dist)

                hit_position = r[3]

                if hit_fraction < 1.0:
                    # hit something → RED
                    p.addUserDebugLine(ray_from[i], hit_position, [1, 0, 0], 1, lifeTime=0.1)
                else:
                    # no hit → GREEN
                    p.addUserDebugLine(ray_from[i], ray_to[i], [0, 1, 0], 1, lifeTime=0.1)

        else:
            print(f"No sensor named: {sensor}")


    def run_simulation(self):
        """
        Function that runs the simulation.
        """
        # build the walls outside of loop
        self.create_walls()
        # use a for loop to control the system
        for i in range (10000):
            #! this if is for the camera speed
            #if i % 10 == 0:
            self.place_sensors()
            # have the physics server step
            p.stepSimulation()
            # if pybullet runs to 240, we slow it down to 1
            time.sleep(1./240.)

        # disconnect when time runs out 
        p.disconnect()
    

    def create_walls(self, thickness=0.5, height=1.0):
        
        # draw for all track points
        i = 0
        while i < len(self.track_points):
            p1 = self.track_points[0]
            p2 = self.track_points[1]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            length = math.sqrt(dx**2 + dy**2)

            if length == 0:
                return
            
            cx = (p1[0] + p2[0]) / 2.0
            cy = (p1[1] + p2[1]) / 2.0

            yaw = math.atan2(dy, dx)
            orientation = p.getQuaternionFromEuler([0, 0, yaw])

            half_extents = [
                length / 2.0,     # along the segment
                thickness,        # wall thickness
                height / 2.0      # half height
            ]

            collision = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=half_extents
            )

            visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=half_extents,
                rgbaColor=[0.9, 0.1, 0.1, 1]
            )

            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision,
                baseVisualShapeIndex=visual,
                basePosition=[cx, cy, height / 2.0],
                baseOrientation=orientation
            )
            
            # update i
            i += 2
        self.created = True

if __name__ == "__main__":
    # define some dummy track points
    track_points = [(0,0), (10,0), (10,10), (0,10)]
    # create object
    simulator = defineSimulation(track_points)
    # create robot
    simulator.spawn_robot("r2d2.urdf")
    # run the sim
    simulator.run_simulation()