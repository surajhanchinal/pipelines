import numpy as np
from analytical_ik import full_ik,position_ik,loaded_function
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from draw_origins import draw_origin,draw_arm
import random
import cloudpickle


loaded_function = None
with open("lambdified_function.pkl", "rb") as f:
    loaded_function = cloudpickle.load(f)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim3d(-1, 1)
ax.set_ylim3d(-1, 1)
ax.set_zlim3d(-1, 1)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Quiver Plot')


def generate_random_poses(num_poses, radius=0.8):
   """
   Generates random poses within a sphere, represented as 4x4 homogeneous transformation matrices.

   Args:
       num_poses (int): Number of poses to generate.
       radius (float, optional): Radius of the sphere. Defaults to 1.0.

   Returns:
       numpy.ndarray: Array of shape (num_poses, 4, 4) containing the generated poses.
   """

   poses = np.zeros((num_poses, 4, 4))

   for i in range(num_poses):
       # Generate random Euler angles (in radians)
       roll = np.random.uniform(0, 2 * np.pi)
       pitch = np.random.uniform(-np.pi / 2, np.pi / 2)  # Constrain pitch to avoid gimbal lock
       yaw = np.random.uniform(0, 2 * np.pi)

       # Generate random translation within the sphere
       x = np.random.uniform(0.1, 1)
       y = np.random.uniform(-0.2, 1)
       z = np.random.uniform(0, 1.2)

       # Ensure the point lies on the sphere
       r = np.sqrt(x**2 + y**2 + z**2)
       x *= (radius/r)
       y *= (radius/r)
       z *= (radius/r)

       # Create rotation matrices
       Rx = np.array([[1, 0, 0],
                      [0, np.cos(roll), -np.sin(roll)],
                      [0, np.sin(roll), np.cos(roll)]])
       Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                      [0, 1, 0],
                      [-np.sin(pitch), 0, np.cos(pitch)]])
       Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])

       # Combine rotation matrices
       R = np.matmul(Rz, np.matmul(Ry, Rx))

       R_new = np.eye(4)
       R_new[:3,:3] = R

       # Create translation matrix
       T = np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])

       # Combine rotation and translation into a homogeneous transformation matrix
       pose = np.matmul(T,R_new)

       poses[i] = pose

   return poses

# Example usage:
num_poses = 10000
poses = generate_random_poses(num_poses)
#print(poses)

import time

azimuth = 0
start = time.time()
'''for i in range(1000):
  sols = position_ik(poses[i][0,3],poses[i][1,3],poses[i][2,3])
  #sols = position_ik(0.4,-0.1,0.7)
  if(len(sols)):
     for sol in sols:
        print([x*(180/np.pi) for x in sol])
        print(poses[i][0,3],poses[i][1,3],poses[i][2,3])
        break
        ax = fig.add_subplot(111, projection='3d')
        ax.view_init(elev=30., azim=azimuth)
        azimuth = (azimuth+1) % 360
        ax.set_xlim3d(-1, 1)
        ax.set_ylim3d(-1, 1)
        ax.set_zlim3d(-1, 1)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Quiver Plot')
        draw_origin(ax,1,np.eye(4))
        draw_arm(ax,0,0,np.pi/2,0,0)
        draw_arm(ax,sol[0],sol[1],sol[2],sol[3],sol[4])
        plt.pause(0.01)
        plt.cla()
        break
  break'''
end = time.time()
print("time taken: ",end-start)


print(loaded_function(-70*(np.pi/180.0), -59.64774709948091*(np.pi/180.0), 89.98889004260712*(np.pi/180.0), 109.65885705687377*(np.pi/180.0), 23.955930404481986*(np.pi/180.0)))