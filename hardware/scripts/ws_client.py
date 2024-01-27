import asyncio
import websockets
import redis.asyncio as Redis

STOPWORD = "STOP"


async def redisReader(channel: Redis.client.PubSub,websocket):
    while True:
        message = await channel.get_message(ignore_subscribe_messages=True)
        if message is not None:
            message_str = message['data'].decode()
            await websocket.send(message_str)
            print(f"(Reader) Message Received: {message}")
            if message["data"].decode() == STOPWORD:
                print("(Reader) STOP")
                break


r = Redis.from_url("redis://localhost")
async def run():
    async with websockets.connect("ws://192.168.0.125:8765") as websocket:
        async with r.pubsub() as pubsub:
            await pubsub.subscribe("channel:1")

            future = asyncio.create_task(redisReader(pubsub,websocket))

            await future
    

if __name__ == '__main__':
    asyncio.run(run())
