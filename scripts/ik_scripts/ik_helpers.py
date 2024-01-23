import matplotlib.pyplot as plt
from draw_origins import draw_arm
import numpy as np

groundHeight = 0
elasticity = 0.6
slowDownFactor = 0.75

class EstimatedParams:
    def __init__(self, x0, y0, z0, vx, vy, vz):
        self.x0 = x0
        self.y0 = y0
        self.z0 = z0
        self.vx = vx
        self.vy = vy
        self.vz = vz

def get_predicted_point_at_time(params, elapsed_time_in_seconds):
    global groundHeight,elasticity,slowDownFactor
    g = 9.8
    b = params.vz
    c = params.z0
    time_to_contact = (b + np.sqrt(b * b + 2 * g * c)) / g
    velocity_at_contact = elasticity * (params.vz - (9.8 * time_to_contact))

    if elapsed_time_in_seconds < time_to_contact:
        x = params.x0 + elapsed_time_in_seconds * params.vx
        y = params.y0 + elapsed_time_in_seconds * params.vy
        z = params.z0 + elapsed_time_in_seconds * params.vz - 4.9 * elapsed_time_in_seconds**2
    else:
        time_from_contact = elapsed_time_in_seconds - time_to_contact
        factor = slowDownFactor
        x = (params.x0 + time_to_contact * params.vx) + time_from_contact * factor * params.vx
        y = (params.y0 + time_to_contact * params.vy) + time_from_contact * factor * params.vy
        z = -(velocity_at_contact * time_from_contact) - (4.9 * time_from_contact**2)

    return x, y, z

def update_trajectory(ax,fig,vx,vy,vz,x0,y0,z0,j1,j2,j3,j4,j5):
    points = []
    ye = y0
    t = 0
    while ye >= 0:
        t = t +  0.0001
        params = EstimatedParams(x0=x0, y0=y0, z0=z0, vx=vx, vy=vy, vz=vz)
        xe,ye,ze = get_predicted_point_at_time(params,t)
        points.append([xe,ye,ze])
    x_coordinates, y_coordinates, z_coordinates = zip(*points)

    ax.clear()
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    draw_arm(ax,j1,j2,j3,j4,j5)
    ax.plot3D(x_coordinates,y_coordinates,z_coordinates, label="Parabolic Trajectory")  # Replot trajectory
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Parabolic Trajectory")
    ax.legend()
    #ax.view_init(elev=15, azim=-60)
    fig.canvas.draw_idle()  # Redraw the figure