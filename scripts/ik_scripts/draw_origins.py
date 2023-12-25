#def draw_arm(j1,j2,j3,j4,j5):
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from fk_numpy import T1,T2,T3,T4,T5

def draw_origin(ax,length,pose_matrix):
  x = np.array([[length],[0],[0],[1]])
  y = np.array([[0],[length],[0],[1]])
  z = np.array([[0],[0],[length],[1]])
  x = np.matmul(pose_matrix,x)
  y = np.matmul(pose_matrix,y)
  z = np.matmul(pose_matrix,z)
  x0 = pose_matrix[0,3]
  y0 = pose_matrix[1,3]
  z0 = pose_matrix[2,3]
  ax.quiver(x0,y0,z0,x[0,0]-x0,x[1,0]-y0,x[2,0]-z0,length=length, color=np.array([0,1,0]))
  ax.quiver(x0,y0,z0,y[0,0]-x0,y[1,0]-y0,y[2,0]-z0,length=length, color=np.array([0,0,1]))
  ax.quiver(x0,y0,z0,z[0,0]-x0,z[1,0]-y0,z[2,0]-z0,length=length, color=np.array([1,0,0]))


def draw_line(ax,x1,y1,z1,x2,y2,z2):
  ax.quiver(x1,y1,z1,x2-x1,y2-y1,z2-z1,length=1, color=np.random.rand(3,))
  


def draw_arm(ax,j1e,j2e,j3e,j4e,j5e):
  #print(j1e*(180/np.pi),j2e*(180/np.pi),j3e*(180/np.pi),j4e*(180/np.pi),j5e*(180/np.pi))
  draw_origin(ax,0.5,np.eye(4))
  T1e = T1(j1e)
  draw_line(ax,0,0,0,T1e[0,3],T1e[1,3],T1e[2,3])
  #draw_origin(ax,0.3,T1e)
  T2e = T1e@T2(j2e)
  draw_line(ax,T1e[0,3],T1e[1,3],T1e[2,3],T2e[0,3],T2e[1,3],T2e[2,3])
  #draw_origin(ax,0.3,T2e)
  T3e = T2e@T3(j3e)
  draw_line(ax,T2e[0,3],T2e[1,3],T2e[2,3],T3e[0,3],T3e[1,3],T3e[2,3])
  #draw_origin(ax,0.3,T3e)
  T4e = T3e@T4(j4e)
  draw_line(ax,T3e[0,3],T3e[1,3],T3e[2,3],T4e[0,3],T4e[1,3],T4e[2,3])
  #draw_origin(ax,0.3,T4e)
  T5e = T4e@T5(j5e)
  draw_line(ax,T4e[0,3],T4e[1,3],T4e[2,3],T5e[0,3],T5e[1,3],T5e[2,3])
  pt2 = np.array([[0],[0],[0.3],[1]])
  tpt2 = T5e @ pt2
  draw_line(ax,T5e[0,3],T5e[1,3],T5e[2,3],tpt2[0,0],tpt2[1,0],tpt2[2,0])
  #draw_origin(ax,0.3,T5e)