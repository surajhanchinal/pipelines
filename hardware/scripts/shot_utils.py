import numpy as np
import math
ms = np.array([-35,-35,-35,-35,-35])
cs = np.array([40000,40000,40000,40000,40000])
cs_final = np.array([4000,4000,4000,20000,20000])
ms_final = np.array([0,0,0,0,0])

def getRealMovements(targets):
    real_targets = np.array([0,0,0,0,0])
    j2_target = targets[1]
    j3_target = targets[2] + j2_target/3
    j4_target = targets[3] - targets[4] + j3_target/3 - j2_target/9
    j5_target = targets[3] + targets[4] + j3_target/3 - j2_target/9
    real_targets[0] = targets[0]
    real_targets[1] = targets[1]
    real_targets[2] = j3_target
    real_targets[3] = j4_target
    real_targets[4] = j5_target
    return real_targets

final_shot_targets = getRealMovements(np.array([0,0,0,0,400]))
normal_time = np.zeros((5,))
initial_shot_time = np.zeros((5,))
final_shot_time = np.zeros((5,))
    
def get_time(targets):
    target1 = targets.copy()
    target2 = targets.copy()
    target2[4] = target2[4] - 200
    target1 = getRealMovements(target1)
    target2 = getRealMovements(target2)
    for i in range(5):
        normal_time[i] = getTimeTaken(ms[i],cs[i],target1[i])
        initial_shot_time[i] = getTimeTaken(ms[i],cs[i],target2[i])
        final_shot_time[i] = getTimeTaken(ms_final[i],cs_final[i],final_shot_targets[i])
    return np.max(normal_time),np.max(initial_shot_time),np.max(final_shot_time)
def get_f(m,c):
    k = c/m
    def f(t,d):
        return k/m*(math.exp(m*t) - 1) - k*t - d/2
    
    return f

def get_df(m,c):
    k = c/m
    def df(t):
        return k*math.exp(m*t) - k
    return df

def getTimeTaken(m,c,steps):
    if(m == 0):
        return 2*np.sqrt(np.abs(steps)/c)
    else:
        f = get_f(m,c)
        df = get_df(m,c)
        return newton(f,df,steps)

def newton(f,Df,steps,x0=0.05,epsilon=0.001,max_iter=100):
    xn = x0
    for n in range(0,max_iter):
        fxn = f(xn,steps/2)
        if math.fabs(fxn) < epsilon:
            return 2*xn
        Dfxn = Df(xn)
        if Dfxn == 0:
            return None
        xn = xn - fxn/Dfxn
    return None


