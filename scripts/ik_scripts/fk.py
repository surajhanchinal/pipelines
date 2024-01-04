from sympy import symbols, Matrix, sin, cos,pprint,simplify,atan,Symbol,solve
import math
import sympy
from cloudpickle import loads, dumps
import cloudpickle
import numpy as np

def rot_x(theta):
    """Returns the 4x4 rotation matrix around the X-axis."""
    return Matrix([
        [1, 0, 0, 0],
        [0, cos(theta), -sin(theta), 0],
        [0, sin(theta), cos(theta), 0],
        [0, 0, 0, 1]
    ])

def rot_y(theta):
    """Returns the 4x4 rotation matrix around the Y-axis."""
    return Matrix([
        [cos(theta), 0, sin(theta), 0],
        [0, 1, 0, 0],
        [-sin(theta), 0, cos(theta), 0],
        [0, 0, 0, 1]
    ])

def rot_z(theta):
    """Returns the 4x4 rotation matrix around the Z-axis."""
    return Matrix([
        [cos(theta), -sin(theta), 0, 0],
        [sin(theta), cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def trans_x(tx):
    """Returns the 4x4 translation matrix along the X-axis."""
    return Matrix([
        [1, 0, 0, tx],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def trans_y(ty):
    """Returns the 4x4 translation matrix along the Y-axis."""
    return Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, ty],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def trans_z(tz):
    """Returns the 4x4 translation matrix along the Z-axis."""
    return Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])

##Final transformation matrix
T_f = Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

### Final pose matrix

## Joint 1. Rotation along z axis
J1_a,J1_Lx,J1_Lz = symbols('J1_a J1_Lx J1_Lz')

T_f = T_f*rot_z(J1_a)
T_f = T_f*trans_z(J1_Lz)
T_f = T_f*trans_x(J1_Lx)

J2_a,J2_Lz = symbols('J2_a J2_Lz')

T_f = T_f*rot_y(J2_a)
T_f = T_f*trans_z(J2_Lz)

## Joint 3, rotation along y axis
J3_a,J3_Lz = symbols('J3_a J3_Lz')

T_f = T_f*rot_y(J3_a)
T_f = T_f*trans_z(J3_Lz)

## Joint 4, rotation along y axis
J4_a,J4_Lz = symbols('J4_a J4_Lz')

T_f = T_f*rot_y(J4_a)
T_f = T_f*trans_z(J4_Lz)

## Joint 5, rotation along z axis
J5_a_final,J5_a,J5_Lx = symbols('J5_a_final J5_a J5_Lx')

T_f = T_f*rot_z(J5_a)
T_f = T_f*trans_x(J5_Lx)
T_f = T_f*rot_x(J5_a_final)
T_f = T_f.subs(sin(J5_a_final),-1)
T_f = T_f.subs(cos(J5_a_final),0)


T_f.simplify()

from sympy.utilities.codegen import codegen
c_code = codegen(('result', T_f), 'C')
print(c_code[0][1])

T_f = T_f.subs({J1_Lx:  0.0525,
J1_Lz: 0.66,
J2_Lz: 0.3,
J3_Lz: 0.3,
J4_Lz: 0.1,
J5_Lx: 0.45,
})

T_func = sympy.lambdify([J1_a,J2_a,J3_a,J4_a,J5_a],T_f,modules=['numpy'])

jj1 = -70.0*(np.pi/180)
jj2 = -59.64774709948091*(np.pi/180)
jj3 = 89.98889004260712*(np.pi/180)
jj4 = 109.65885705687377*(np.pi/180)
jj5 = 23.955930404481986*(np.pi/180)


pprint(T_f.subs({J1_a:jj1,J2_a:jj2,J3_a:jj3,J4_a:jj4,J5_a:jj5}).evalf())
print("   ")
print(T_func(jj1,jj2,jj3,jj4,jj5))

my_subs = {J1_Lx:  0.0525,
J1_Lz: 0.66,
J2_Lz: 0.3,
J3_Lz: 0.3,
J4_Lz: 0.1,
J5_Lx: 0.45,
}

def getf(j1a,j2a,j3a,j4a,j5a):
    #s1 = J1_Lz
    #s2 = s1 + J2_Lz*np.cos(j2a)
    #s3 = s2 + J3_Lz*np.cos(j2a + j3a)
    #s4 =s3 + J4_Lz*np.cos(j2a + j3a + j4a)
    #s5 = s4 - J5_Lx*np.sin(j2a + j3a + j4a)*np.cos(j5a)
    e1 = J5_Lx
    e2 = np.sin(j2a+j3a+j4a)
    e3 = np.cos(j5a)
    print(e1.subs(my_subs).evalf(),e2,e3)
    #print(s1.subs(my_subs).evalf(),s2.subs(my_subs).evalf(),s3.subs(my_subs).evalf(),s4.subs(my_subs).evalf(),s5.subs(my_subs).evalf())
    return J1_Lz + J2_Lz*np.cos(j2a) + J3_Lz*np.cos(j2a + j3a) + J4_Lz*np.cos(j2a + j3a + j4a) - J5_Lx*np.sin(j2a + j3a + j4a)*np.cos(j5a)

print(getf(jj1,jj2,jj3,jj4,jj5).subs({J1_Lx:  0.0525,
J1_Lz: 0.66,
J2_Lz: 0.3,
J3_Lz: 0.3,
J4_Lz: 0.1,
J5_Lx: 0.45,
}).evalf())


with open('lambdified_function.pkl', 'wb') as f_out:
    cloudpickle.dump(T_func, f_out)