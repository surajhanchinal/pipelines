import serial
import websockets
import asyncio
import time
from shot_utils import get_time
import numpy as np

final_shot_string = "SA 3 0 40000 | SA 4 0 40000 | MR 0 0 0 0 400"
def parseMessage(message):
    split_msg = message.split()
    if(len(split_msg) != 7 or split_msg[0] != "MRT"):
        return False,None,None,None
    time_to_contact_ns = int(split_msg[6])
    single_frame_time,initial_time,final_shot_time = get_time(np.array([int(split_msg[j]) for j in range(1,6)]))
    time_remaining_ns = time_to_contact_ns - time.time_ns()
    time_remaining_s = time_remaining_ns/1e9
    print(time_remaining_s,initial_time,final_shot_time)
    if(time_remaining_s < initial_time + final_shot_time):
        return False,"SA 3 0 4000 | SA 4 0 4000 | MR {j1} {j2} {j3} {j4} {j5}".format(j1=split_msg[1],j2=split_msg[2],j3=split_msg[3],j4=split_msg[4],j5=split_msg[5]),None,None
    else:
        delay = time_remaining_s - initial_time - final_shot_time
        return True,"SA 3 0 4000 | SA 4 0 4000 | MR {j1} {j2} {j3} {j4} {j5}".format(j1=split_msg[1],j2=split_msg[2],j3=split_msg[3],j4=split_msg[4],j5=(int(split_msg[5]) - 200)),final_shot_string,delay

async def serial_writer(websocket,ser):
    while True:
        message = await websocket.recv()
        print("message: ",message)
        twoShots,mv1,mv2,delay = parseMessage(message)
        if(not twoShots and mv1 is None):
            print("sending normal message")
            ser.write(message.encode() + b'\n')
        elif(not twoShots and not (mv1 is None)):
            print("sending single shot  message")
            ser.write(mv1.encode() + b'\n')
        elif(twoShots):
            print("sending double shot  message with delay: ",delay)
            print("sent",mv1)
            ser.write(mv1.encode() + b'\n')
            await asyncio.sleep(delay)
            print("sent",mv2)
            ser.write(mv2.encode() + b'\n')
        await asyncio.sleep(0)

async def serial_reader(websocket,ser):
    while True:
        if ser.in_waiting > 0:
            message_str = ser.readline().decode().strip('\n')
            await websocket.send(message_str.strip('\n'))
        await asyncio.sleep(0)


async def run():
    async with websockets.connect("ws://192.168.0.125:8765") as websocket:
        with serial.Serial("/dev/ttyACM0") as ser:
            future1 = asyncio.create_task(serial_reader(websocket,ser))
            future2 = asyncio.create_task(serial_writer(websocket,ser))
            await future1
            await future2


if __name__ == "__main__":
    asyncio.run(run())