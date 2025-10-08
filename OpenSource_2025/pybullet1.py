import pybullet as p
import pybullet_data
import time

# --- Setup ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)

plane = p.loadURDF("plane.urdf")

# Load the KUKA iiwa arm
kuka = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

# --- Control Parameters ---
joint_indices = list(range(7))  # KUKA has 7 revolute joints
joint_velocity = 0.02  # change per keypress

print("Control joints 0-6 using keys:")
print("p/u = joint 0 up/down")
print("o/i = joint 1 up/down")
print("t/y = joint 2 up/down")
print("r/e = joint 3 up/down")
print("l/k = joint 4 up/down")
print("j/h = joint 5 up/down")
print("g/f = joint 6 up/down")
print("ESC = quit")

# --- Simulation loop ---
joint_positions = [0] * 7  # initial joint angles

while True:
    keys = p.getKeyboardEvents()

    # Quit
    if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
        break

    # --- Map keys to joint movements ---
    if ord('p') in keys and keys[ord('p')] & p.KEY_IS_DOWN:  joint_positions[0] += joint_velocity
    if ord('u') in keys and keys[ord('u')] & p.KEY_IS_DOWN:  joint_positions[0] -= joint_velocity

    if ord('o') in keys and keys[ord('o')] & p.KEY_IS_DOWN:  joint_positions[1] += joint_velocity
    if ord('i') in keys and keys[ord('i')] & p.KEY_IS_DOWN:  joint_positions[1] -= joint_velocity

    if ord('t') in keys and keys[ord('t')] & p.KEY_IS_DOWN:  joint_positions[2] += joint_velocity
    if ord('y') in keys and keys[ord('y')] & p.KEY_IS_DOWN:  joint_positions[2] -= joint_velocity

    if ord('r') in keys and keys[ord('r')] & p.KEY_IS_DOWN:  joint_positions[3] += joint_velocity
    if ord('e') in keys and keys[ord('e')] & p.KEY_IS_DOWN:  joint_positions[3] -= joint_velocity

    if ord('l') in keys and keys[ord('l')] & p.KEY_IS_DOWN:  joint_positions[4] += joint_velocity
    if ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN:  joint_positions[4] -= joint_velocity

    if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN:  joint_positions[5] += joint_velocity
    if ord('h') in keys and keys[ord('h')] & p.KEY_IS_DOWN:  joint_positions[5] -= joint_velocity

    if ord('g') in keys and keys[ord('g')] & p.KEY_IS_DOWN:  joint_positions[6] += joint_velocity
    if ord('f') in keys and keys[ord('f')] & p.KEY_IS_DOWN:  joint_positions[6] -= joint_velocity

    # --- Apply positions ---
    for i, joint_index in enumerate(joint_indices):
        p.setJointMotorControl2(
            bodyIndex=kuka,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_positions[i],
            force=500
        )

    p.stepSimulation()
    time.sleep(1/240)
