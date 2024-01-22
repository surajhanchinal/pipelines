import numpy as np
import datetime
import time
unix_ns = 1705772559465779908
unix_ns2 = 1705772558784779908
x = np.datetime64(unix_ns,'ns')
y = np.datetime64(unix_ns2,'ns')
print((1705773041977120205 - time.time_ns()))
print(time.time_ns())