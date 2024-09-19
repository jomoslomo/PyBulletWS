import pybullet as p
import pybullet_data
import time
import math
import asyncio
import websockets
import json

# WebSocket server URL
SERVER_URL = "ws://localhost:8080"

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

    # Function to get joint information
    def get_joint_info():
        joint_info = []
        for i in range(num_joints):
            joint_state = p.getJointState(robot_arm_id, i)
            joint_position = joint_state[0]
            joint_name = p.getJointInfo(robot_arm_id, i)[1].decode('utf-8')
            joint_info.append({
                "name": joint_name,
                "position": joint_position,
                "position_degrees": math.degrees(joint_position)
            })
        return joint_info

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
        
        # Return the updated joint information
        return get_joint_info()

    # Function to handle commands
    def handle_command(command):
        global target_position
        step = 0.05
        if command == 'up':
            target_position[2] += step
        elif command == 'down':
            target_position[2] -= step
        elif command == 'left':
            target_position[1] -= step
        elif command == 'right':
            target_position[1] += step
        elif command == 'w':
            target_position[0] += step
        elif command == 's':
            target_position[0] -= step
        print(f"New target position: {target_position}")
        return set_target_position(target_position)

    # WebSocket client
    async def websocket_client():
        async with websockets.connect(SERVER_URL) as websocket:
            print(f"Connected to WebSocket server at {SERVER_URL}")
            while True:
                try:
                    command = await websocket.recv()
                    print(f"Received command: {command}")
                    joint_info = handle_command(command)
                    # Send joint information back to the server
                    await websocket.send(json.dumps({"type": "joint_info", "data": joint_info}))
                except websockets.exceptions.ConnectionClosed:
                    print("WebSocket connection closed, reconnecting...")
                    break

    # Add a visual marker for the target position
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 0.7])
    target_marker = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=target_position)

    # Start WebSocket client in a separate thread
    import threading
    websocket_thread = threading.Thread(target=asyncio.run, args=(websocket_client(),))
    websocket_thread.start()

    # Initial print of joint rotations
    set_target_position(target_position)

    print("WebSocket client started. Waiting for commands...")
    print("Use the WebSocket server to send 'up', 'down', 'left', 'right', 'w', 's' commands.")

    # Simulation loop
    while True:
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