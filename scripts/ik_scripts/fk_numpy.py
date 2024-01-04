import numpy as np

J1_Lx =  0.0525
J1_Lz = 0.66
J2_Lz = 0.3
J3_Lz = 0.3
J4_Lz = 0.1
J5_Lx = 0.45

def rot_x(theta):
    """Returns the 4x4 rotation matrix around the X-axis."""
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(theta), -np.sin(theta), 0],
        [0, np.sin(theta), np.cos(theta), 0],
        [0, 0, 0, 1]
    ])

def rot_y(theta):
    """Returns the 4x4 rotation matrix around the Y-axis."""
    return np.array([
        [np.cos(theta), 0, np.sin(theta), 0],
        [0, 1, 0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [0, 0, 0, 1]
    ])

def rot_z(theta):
    """Returns the 4x4 rotation matrix around the Z-axis."""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def trans_x(tx):
    """Returns the 4x4 translation matrix along the X-axis."""
    return np.array([
        [1, 0, 0, tx],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def trans_y(ty):
    """Returns the 4x4 translation matrix along the Y-axis."""
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, ty],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def trans_z(tz):
    """Returns the 4x4 translation matrix along the Z-axis."""
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])


def T1(j1):
    return (rot_z(j1) @trans_z(J1_Lz))@trans_x(J1_Lx)

def T2(j2):
    return rot_y(j2) @ trans_z(J2_Lz)

def T3(j3):
    return rot_y(j3) @ trans_z(J3_Lz)

def T4(j4):
    return rot_y(j4) @ trans_z(J4_Lz)

def T5(j5):
    return (rot_z(j5)@trans_x(J5_Lx))@rot_x(-np.pi/2)

