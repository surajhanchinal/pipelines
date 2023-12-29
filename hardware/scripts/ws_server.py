import asyncio
import websockets

connected = set()

async def chat(websocket, path):
    connected.add(websocket)
    try:
        async for message in websocket:
            print(message)
            for conn in connected:
                if conn != websocket:
                    await conn.send(message)
    finally:
        connected.remove(websocket)

async def main():
    async with websockets.serve(chat, "localhost", 8765):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())