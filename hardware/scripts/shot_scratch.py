import numpy as np
from shot_utils import get_time
import time
t1 = time.time_ns()
single_frame_time,initial_time,final_shot_time = get_time(np.array([800,-300,400,200,900]))
t2 = time.time_ns()
diff = t2 - t1
diff = diff/1000
print(diff)