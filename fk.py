import pybullet as p
import pybullet_data
import time
import keyboard

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

    # Initialize joint positions
    joint_positions = [0] * num_joints

    # Define a dictionary to map keys to joint indices and movements
    key_bindings = {
        '1': (0, 0.1, "Joint 0 positive"),
        '2': (0, -0.1, "Joint 0 negative"),
        '3': (1, 0.1, "Joint 1 positive"),
        '4': (1, -0.1, "Joint 1 negative"),
        '5': (2, 0.1, "Joint 2 positive"),
        '6': (2, -0.1, "Joint 2 negative"),
    }

    print("Use the following keys to control the robot:")
    print("1/2: Joint 0 positive/negative")
    print("3/4: Joint 1 positive/negative")
    print("5/6: Joint 2 positive/negative")
    print("Press 'q' to quit the simulation")

    # Function to handle key press
    def on_key_press(event):
        if event.name in key_bindings:
            joint, delta, movement_desc = key_bindings[event.name]
            joint_positions[joint] += delta
            print(f"Key pressed: {movement_desc}")

    # Register the key press handler
    for key in key_bindings:
        keyboard.on_press_key(key, on_key_press)

    # Simulation loop
    while True:
        if keyboard.is_pressed('q'):  # Press 'q' to quit
            print("Quitting simulation...")
            break

        # Apply joint positions
        for i in range(num_joints):
            p.setJointMotorControl2(
                bodyUniqueId=robot_arm_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_positions[i],
                force=500
            )
        
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