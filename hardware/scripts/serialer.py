import serial
import websockets
import asyncio
import time
from shot_utils import get_time
import numpy as np

final_shot_string = "MR 0 0 0 0 400"

def parseMessage(message):
    split_msg = message.split()
    if(len(split_msg) != 7 and split_msg[0] != "MRT"):
        return False,None,None,None
    time_to_contact_ns = split_msg[6]
    single_frame_time,initial_time,final_shot_time = get_time(np.array([int(split_msg[j]) for j in range(1,6)]))
    time_remaining_ns = time_to_contact_ns - time.time_ns()
    if(time_remaining_ns > initial_time + final_shot_time):
        return False,"MR {j1} {j2} {j3} {j4} {j5}".format(j1=split_msg[1],j2=split_msg[2],j3=split_msg[3],j4=split_msg[4],j5=split_msg[5]),None,None
    else:
        delay = initial_time + final_shot_time - time_remaining_ns
        return True,"MR {j1} {j2} {j3} {j4} {j5}".format(j1=split_msg[1],j2=split_msg[2],j3=split_msg[3],j4=split_msg[4],j5=(int(split_msg[5]) - 200)),final_shot_string,delay

async def serial_writer(websocket,ser):
    while True:
        message = await websocket.recv()
        twoShots,mv1,mv2,delay = parseMessage(message)
        if(not twoShots and mv1 is None):
            ser.write(message.encode() + b'\n')
        elif(not twoShots and not (mv1 is None)):
            ser.write(mv1.encode() + b'\n')
        elif(twoShots):
            ser.write(mv1.encode() + b'\n')
            await asyncio.sleep(delay)
            ser.write(mv2.encode() + b'\n')
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