import numpy as np
from shot_utils import get_time
import time
t1 = time.time_ns()
single_frame_time,initial_time,final_shot_time = get_time(np.array([800,-300,400,200,900]))
s1 = (20/(single_frame_time+0.140))*3.6
s2 = (20/(initial_time+final_shot_time+0.140))*3.6
print(s1,s2)
t2 = time.time_ns()
diff = t2 - t1
diff = diff/1000
print(diff)