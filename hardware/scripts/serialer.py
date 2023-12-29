import serial
import websockets
import asyncio


async def serial_writer(websocket,ser):
    while True:
        message = await websocket.recv()
        ser.write(message.encode() + b'\n')
        print("wrote to serial")
        await asyncio.sleep(0)


async def serial_reader(websocket,ser):
    while True:
        if ser.in_waiting > 0:
            message_str = ser.readline().decode()
            await websocket.send(message_str.strip('\n'))
        await asyncio.sleep(0)


async def run():
    async with websockets.connect("ws://localhost:8765") as websocket:
        with serial.Serial("/dev/ttyACM0") as ser:
            future1 = asyncio.create_task(serial_reader(websocket,ser))
            future2 = asyncio.create_task(serial_writer(websocket,ser))
            await future1
            await future2


if __name__ == "__main__":
    asyncio.run(run())