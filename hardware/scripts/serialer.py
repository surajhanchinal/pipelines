import serial
import websockets
import asyncio
import time
from shot_utils import get_time
import numpy as np


def parseMessage(message):
    split_msg = message.split()
    if(len(split_msg) != 7 and split_msg[0] != "MRT"):
        return False
    time_to_contact_ns = split_msg[6]
    time_remaining_ns = time_to_contact_ns - time.time_ns()
    t1 = time.time_ns()
    single_frame_time,initial_time,final_shot_time = get_time(np.array([int(split_msg[j]) for j in range(1,6)]))
    t2 = time.time_ns()

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