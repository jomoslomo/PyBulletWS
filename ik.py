import pybullet as p
import pybullet_data
import time
import keyboard
import math

try:
    # Connect to PyBullet physics server in GUI mode
    physicsClient = p.connect(p.GUI)

    # Set the additional search path for PyBullet's built-in data
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load a ground plane from PyBullet's built-in data
    planeId = p.loadURDF("plane.urdf")

    # Set the gravity for the simulation
    p.setGravity(0, 0, -9.81)

    # Load your robotic arm URDF file
    robot_arm_id = p.loadURDF("cobot.urdf", useFixedBase=True)

    # Set the simulation step frequency
    simulation_frequency = 240.0  # Hz

    # Reset the camera for a better view
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0, 0, 0])

    # Get the number of joints in the robot
    num_joints = p.getNumJoints(robot_arm_id)

    # Get the end effector link index (assuming it's the last link)
    end_effector_index = num_joints - 1

    # Initialize target position
    target_position = [0.3, 0, 0.5]  # Example target position

    # Function to perform inverse kinematics and print joint rotations
    def set_target_position(target_pos):
        joint_poses = p.calculateInverseKinematics(robot_arm_id, end_effector_index, target_pos)
        print("\nJoint rotations:")
        for i in range(num_joints):
            p.setJointMotorControl2(bodyIndex=robot_arm_id,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=joint_poses[i],
                                    force=500)
            # Get joint info
            joint_info = p.getJointInfo(robot_arm_id, i)
            joint_name = joint_info[1].decode('utf-8')
            # Get current joint state
            joint_state = p.getJointState(robot_arm_id, i)
            joint_position = joint_state[0]
            print(f"  {joint_name}: {joint_position:.4f} radians ({math.degrees(joint_position):.4f} degrees)")

    # Function to handle key press for IK control
    def on_key_press(event):
        global target_position
        step = 0.05
        if event.name == 'up':
            target_position[2] += step
        elif event.name == 'down':
            target_position[2] -= step
        elif event.name == 'left':
            target_position[1] -= step
        elif event.name == 'right':
            target_position[1] += step
        elif event.name == 'w':
            target_position[0] += step
        elif event.name == 's':
            target_position[0] -= step
        print(f"New target position: {target_position}")
        set_target_position(target_position)

    # Register the key press handler
    keyboard.on_press_key('up', on_key_press)
    keyboard.on_press_key('down', on_key_press)
    keyboard.on_press_key('left', on_key_press)
    keyboard.on_press_key('right', on_key_press)
    keyboard.on_press_key('w', on_key_press)
    keyboard.on_press_key('s', on_key_press)

    print("Use the following keys to control the robot:")
    print("Up/Down: Move end effector up/down")
    print("Left/Right: Move end effector left/right")
    print("W/S: Move end effector forward/backward")
    print("Press 'q' to quit the simulation")

    # Add a visual marker for the target position
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 0.7])
    target_marker = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=target_position)

    # Initial print of joint rotations
    set_target_position(target_position)

    # Simulation loop
    while True:
        if keyboard.is_pressed('q'):  # Press 'q' to quit
            print("Quitting simulation...")
            break

        # Update the position of the target marker
        p.resetBasePositionAndOrientation(target_marker, target_position, [0, 0, 0, 1])
        
        # Step the simulation
        p.stepSimulation()
        
        # Sleep to control simulation speed
        time.sleep(1.0 / simulation_frequency)

except p.error as e:
    print(f"An error occurred: {e}")

finally:
    # Disconnect from PyBullet
    if p.isConnected():
        p.disconnect()
    print("Disconnected from PyBullet")