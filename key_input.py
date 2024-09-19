import asyncio
import websockets
import keyboard

# WebSocket server URL
SERVER_URL = "ws://localhost:8080"

async def send_input():
    async with websockets.connect(SERVER_URL) as websocket:
        print(f"Connected to WebSocket server at {SERVER_URL}")
        print("Use arrow keys, 'w', and 's' to control the robot. Press 'q' to quit.")

        last_sent = set()  # To keep track of the last sent keys

        while True:
            current_keys = set()
            
            # Check the state of each key
            for key in ['up', 'down', 'left', 'right', 'w', 's']:
                if keyboard.is_pressed(key):
                    current_keys.add(key)

            # Send updates only if the state has changed
            if current_keys != last_sent:
                for key in current_keys:
                    print(f"Sending command: {key}")
                    await websocket.send(key)
                last_sent = current_keys

            # Check for quit command
            if keyboard.is_pressed('q'):
                print("Quitting...")
                break

            # Small delay to prevent overwhelming the server
            await asyncio.sleep(0.05)

async def main():
    try:
        await send_input()
    except websockets.exceptions.ConnectionClosed:
        print("Connection to the server was closed.")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    asyncio.run(main())