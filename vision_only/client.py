import asyncio
import websockets
import json

async def receive_position():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        while True:
            message = await websocket.recv()
            pose = json.loads(message)
            print(f"Received pose: {pose}")

if __name__ == "__main__":
    asyncio.run(receive_position())