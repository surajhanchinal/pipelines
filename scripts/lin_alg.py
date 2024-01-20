import numpy as np
from scipy.spatial.transform import Rotation   


a1 = np.array([-0.513,-0.0339,5.753])
b1 = np.array([0.582,-0.059,5.804])
c1 = np.array([0.598,1.3617,5.916])
d1 = np.array([-0.496,1.376,5.830])

a2 = np.array([-0.551,-0.094,6.555])
b2 = np.array([0.549,-0.122,6.705])
c2 = np.array([0.57,1.302,6.743])
d2 = np.array([-0.543,1.314,6.639])

## y diff

o1 = d1 - a1
i1 = np.array([0,1.035,0])

o2 = c1 - b1
i2 = np.array([0,1.035,0])

o3 = d2 - a2
i3 = np.array([0,1.035,0])

o4 = c2 - b2
i4 = np.array([0,1.035,0])

## x diff

o5 = b1 - a1
i5 = np.array([0.995,0,0])

o6 = c1 - d1
i6 = np.array([0.995,0,0])

o7 = b2 - a2
i7 = np.array([0.995,0,0])

o8 = c2 - d2
i8 = np.array([0.995,0,0])

## Z diff


o9 = a2 - a1
i9 = np.array([0,0,0.644])

o10 = b2 - b1
i10 = np.array([0,0,0.644])

o11 = c2 - c1
i11 = np.array([0,0,0.644])

o12 = d2 - d1
i12 = np.array([0,0,0.644])

#Ax = b

A = np.array([i1,i2,i3,i4,i5,i6,i7,i8,i9,i10,i11,i12])
b = np.array([o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11,o12])



at = A.transpose()
ata = A.transpose() @ A

ata_inv = np.linalg.inv(ata)

atb = at @ b

x = ata_inv @ atb
print(x)


### first transform the matrix to euler angles
r =  Rotation.from_matrix(x.transpose())
angles = r.as_euler("zyx",degrees=True)
r1 = Rotation.from_euler("zyx",angles,degrees=True)
print(angles)

print(x.transpose())
print(r.as_matrix())
print(r1.as_matrix())


### x diff
## -xi*sin(y) - zo should be zero
print(o5[2])
print(o6[2])
print(o7[2])
print(o8[2])

theta_y = (np.arcsin(-(o5[2]/i5[0]))*(180/np.pi) + np.arcsin(-(o6[2]/i6[0]))*(180/np.pi) )/2

ty_r = theta_y*(np.pi/180)

print(np.arctan(o5[1]/(o5[0]*np.tan(ty_r)))*(180/np.pi))
print(np.arctan(o6[1]/(o6[0]*np.tan(ty_r)))*(180/np.pi))
print(np.arctan(o7[1]/(o7[0]*np.tan(ty_r)))*(180/np.pi))
print(np.arctan(o8[1]/(o8[0]*np.tan(ty_r)))*(180/np.pi))



### theta_x

sx1 = np.arcsin(o1[2]/(i1[1]*np.cos(ty_r)))*(180/np.pi)
sx2 = np.arcsin(o2[2]/(i2[1]*np.cos(ty_r)))*(180/np.pi)
sx3 = np.arcsin(o3[2]/(i3[1]*np.cos(ty_r)))*(180/np.pi)
sx4 = np.arcsin(o4[2]/(i4[1]*np.cos(ty_r)))*(180/np.pi)
print(sx1,sx2,sx3,sx4)
