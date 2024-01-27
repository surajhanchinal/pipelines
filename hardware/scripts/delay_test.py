import numpy as np
import redis
import time

r = redis.from_url("redis://localhost")

#for i in range(1000):
r.publish("channel:1","MRT 0 0 -900 0 0 {t}".format(t=str(time.time_ns() + 2000000000))) 
#time.sleep(1)