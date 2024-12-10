# server.py

import asyncio
import websockets
import json
from data_parser import get_next_data

async def send_data(websocket, path):  # Include the 'path' parameter
    while True:
        data = get_next_data()
        if data:
            print(f"Sending data: {data}")
            await websocket.send(json.dumps(data))
        else:
            # No new data, wait a bit before checking again
            await asyncio.sleep(1)

async def main():
    # Start the WebSocket server
    async with websockets.serve(send_data, "0.0.0.0", 5050, ):
        print("WebSocket server started at ws://0.0.0.0:5050")
        await asyncio.Future()  # Keep the server running indefinitely

if __name__ == "__main__":
    asyncio.run(main())
