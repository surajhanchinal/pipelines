import numpy as np
import cloudpickle

J1_Lx =  0.1
J1_Lz = 0.66
J2_Lz = 0.3
J3_Lz = 0.3
J4_Lz = 0.1
J5_Lx = 0.45

loaded_function = None
with open("lambdified_function.pkl", "rb") as f:
    loaded_function = cloudpickle.load(f)


#not defined when j1 is 90 degrees. Treat this case. Use the Y equation in that case :)
def c1_1(j1e,j5e,j234e,Px):
    return J1_Lx + J4_Lz*np.sin(j234e) - J5_Lx*np.sin(j5e)*np.tan(j1e) + J5_Lx*np.cos(j5e)*np.cos(j234e) - Px/np.cos(j1e)

#not defined when j1 is 0 degrees. Treat this case. Use the Y equation in that case :)
def c1_2(j1e,j5e,j234e,Py):
    return J1_Lx + J4_Lz*np.sin(j234e) + J5_Lx*np.sin(j5e)/np.tan(j1e) + J5_Lx*np.cos(j5e)*np.cos(j234e) - Py/np.sin(j1e)


## Robust to j1
def c1(j1e,j5e,j234e,Px,Py):
    if((np.abs(j1e) - np.pi/2) < 0.05):
        return c1_2(j1e,j5e,j234e,Py)
    return c1_1(j1e,j5e,j234e,Px)

def c2(j5e,j234e,Pz):
    return J1_Lz + J4_Lz*np.cos(j234e) - J5_Lx*np.sin(j234e)*np.cos(j5e) - Pz


def k1():
    return J2_Lz

def k2():
    return J3_Lz

'''
k1*cos(j2) + k2*cos(j2+j3) + c2 = 0 ## x
k1*sin(j2) + k2*sin(j2+j3) + c1 = 0 ## y
'''

def j3(j1e,j5e,j234e,Px,Py,Pz):
    c1_e = c1(j1e,j5e,j234e,Px,Py)
    c2_e = c2(j5e,j234e,Pz)
    k1_e = k1()
    k2_e = k2()
    sol = np.arccos((c1_e*c1_e + c2_e*c2_e - k1_e*k1_e - k2_e*k2_e)/2*k1_e*k2_e)
    return [sol,-sol] 

## Always use the first solution of j3 as parameter
def j2(j3e,j1e,j5e,j234e,Px,Py,Pz):
    c1_e = c1(j1e,j5e,j234e,Px,Py)
    c2_e = c2(j5e,j234e,Pz)
    k1_e = k1()
    k2_e = k2()
    t1 = np.arctan(c1_e/c2_e)
    t2 = np.arctan(k2_e*np.sin(j3e)/(k1_e + k2_e*np.cos(j3e)))
    sol1  =  t1 - t2
    sol2  = t1 + t2
    return [sol1,sol2]

def j1(tyy,txy):
    return np.arctan(tyy/txy)

def j5(tzz,tzx):
    return -np.arctan(tzz/tzx)

def j5_new(j1,Px,Py):
    return np.arcsin((-Px*np.sin(j1) + Py*np.cos(j1))/J5_Lx)

def j234(tzx,tzy,j5e):
    return np.arctan(tzx/(tzy*np.cos(j5e)))

# want the minimal describing angle which describes as an offset of zero, so ideally between -180 to 180
def get_optimal_angle(angle):
    a1 = angle
    a1d = abs(angle)
    a2 = angle + 2*np.pi
    a2d = abs(a2)
    a3 = angle - 2*np.pi
    a3d = abs(a3)
    if a1d <= a2d:
        if a1d <= a3d:
            return a1
        else:
            return a3
    else:
        if a2d <= a3d:
            return a2
        else:
            return a3

def check_full_solution(j1e,j2e,j3e,j4e,j5e,pose_matrix):
    return (np.abs(loaded_function(j1e,j2e,j3e,j4e,j5e) - pose_matrix) < 0.01).all()

def check_position_solution(j1e,j2e,j3e,j4e,j5e,Px,Py,Pz):
    estimated_matrix = loaded_function(j1e,j2e,j3e,j4e,j5e)
    Pxe = np.abs(estimated_matrix[0,3] - Px)
    Pye = np.abs(estimated_matrix[1,3] - Py)
    Pze = np.abs(estimated_matrix[2,3] - Pz)
    return Pxe < 0.01 and Pye < 0.01 and Pze < 0.01


def partial_ik(j1e,j234e,j5e,Px,Py,Pz):
    sols = []

    j234_vals = [j234e]
    if(j234e > 0):
        j234_vals.append(j234e-np.pi)
    else:
        j234_vals.append(j234e+np.pi)
    
    for j234_g in j234_vals:
        [j3_e1,j3_e2] = j3(j1e,j5e,j234_g,Px,Py,Pz)
        [j2_e1,j2_e2] = j2(j3_e1,j1e,j5e,j234_g,Px,Py,Pz)

        j4_e1 = j234_g - j3_e1 - j2_e1
        j4_e2 = j234_g - j3_e2 - j2_e2

        sols.append([get_optimal_angle(j1e),get_optimal_angle(j2_e1),get_optimal_angle(j3_e1),get_optimal_angle(j4_e1),get_optimal_angle(j5e)])
        sols.append([get_optimal_angle(j1e),get_optimal_angle(j2_e2),get_optimal_angle(j3_e2),get_optimal_angle(j4_e2),get_optimal_angle(j5e)])
    
    return sols

def full_ik(pose_matrix):
    txx = pose_matrix[0,0]
    txy = pose_matrix[0,1]
    txz = pose_matrix[0,2]
    tyx = pose_matrix[1,0]
    tyy = pose_matrix[1,1]
    tyz = pose_matrix[1,2]
    tzx = pose_matrix[2,0]
    tzy = pose_matrix[2,1]
    tzz = pose_matrix[2,2]
    Px = pose_matrix[0,3]
    Py = pose_matrix[1,3]
    Pz = pose_matrix[2,3]

    j1e = j1(tyy,txy)
    j5e = j5(tzz,tzx)
    j234e = j234(tzx,tzy,j5e)
    sols = partial_ik(j1e,j234e,j5e,Px,Py,Pz)
    filtered_sols = []
    for sol in sols:
        if(check_full_solution(sol[0],sol[1],sol[2],sol[3],sol[4],pose_matrix)):
            filtered_sols.append(sol)

def position_ik(Px,Py,Pz):
    final_sols = []
    for j1_d in range(-90,100,10):
        for j234_d in range(-90,100,10):
            j1e = j1_d*(np.pi/180)
            j234e = j234_d*(np.pi/180)
            j5e = j5_new(j1e,Px,Py)
            sols = partial_ik(j1e,j234e,j5e,Px,Py,Pz)
            for sol in sols:
                if(check_position_solution(sol[0],sol[1],sol[2],sol[3],sol[4],Px,Py,Pz)):
                    final_sols.append(sol)
    
    return final_sols
