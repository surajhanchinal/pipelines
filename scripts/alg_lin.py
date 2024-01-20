from sympy import symbols, Matrix, sin, cos,pprint,simplify,atan,Symbol,solve
import math
import sympy

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


xi , yi , zi = symbols('xi yi zi')
mi = Matrix([xi,yi,zi,1])
xo , yo , zo = symbols('xo yo zo')
mo = Matrix([xo,yo,zo,1])

theta_x,theta_y,theta_z = symbols('theta_x theta_y theta_z')
rot_m = rot_x(theta_x) * rot_y(theta_y) * rot_z(theta_z)


eq = rot_m * mi - mo
print(eq.shape)
###

eq_x = eq.subs({yi: 0,zi: 0})
print(eq_x)
print("\n\n\n\n")

eq_y = eq.subs({xi: 0,zi: 0})
print(eq_y)
print("\n\n\n\n")
eq_z = eq.subs({yi: 0,xi: 0})
print(eq_z)