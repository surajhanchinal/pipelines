import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider,Button
import numpy as np
from draw_origins import draw_arm
import time
import redis

# Connect to the Redis server
redis_client = redis.StrictRedis(host='localhost', port=6379, db=0)

# Create a pubsub object
pubsub = redis_client.pubsub()


redis_client2 = redis.StrictRedis(host='localhost', port=6379, db=0)

# Note: The above loop will run indefinitely, listening for messages on the subscribed channel.
# You may want to run this in a separate thread or process if you need to perform other tasks.


# Define initial point and velocity vector
x0 = 0.4
y0 = 20
z0 = 1.5
velocity_vector = [0, 30, 0]

# Define gravitational constant and time step
g = 9.81
dt = 0.1


# Create the figure and 3D plot
fig, ax = plt.subplots(figsize=(8, 6), subplot_kw={"projection": "3d"})
j1 = (-666/1000)*(90)*(np.pi/180)
j2 = (13/900)*(90)*(np.pi/180)
j3 = (899/900)*(90)*(np.pi/180)
j4 = (-803/900)*(90)*(np.pi/180)
j5 = (-44/900)*(90)*(np.pi/180)

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
        print(elapsed_time_in_seconds,time_from_contact)
        z = -(velocity_at_contact * time_from_contact) - (4.9 * time_from_contact**2)

    return x, y, z

def handler(message):
    global j1,j2,j3,j4,j5
    #print(message)
    angles = message['data'].split()
    angles = [float(a) for a in angles]
    j1 = angles[0]
    j2 = angles[1]
    j3 = angles[2]
    j4 = angles[3]
    j5 = angles[4]
    plt.cla()
    print(j1,j2,j3,j4,j5)
    update_trajectory(None)
# Subscribe to a channel
pubsub.subscribe(**{'solution': handler})
pubsub.run_in_thread(sleep_time=0.01)

def sendMessage(val):
    global velocity_vector,x0,y0,z0
    redis_client2.publish('traj-channel',"{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}".format(velocity_vector[0],-velocity_vector[1],velocity_vector[2],x0,y0,z0))

# Define a function to update the trajectory based on velocity
def update_trajectory(val):
    global velocity_vector,x0,y0,z0
    velocity_vector[0] = vx_slider.val
    velocity_vector[1] = vy_slider.val
    velocity_vector[2] = vz_slider.val
    x0 = x0_slider.val
    z0 = z0_slider.val


    points = []
    ye = y0
    t = 0
    while ye >= 0:
        t = t +  0.0001
        params = EstimatedParams(x0=x0, y0=y0, z0=z0, vx=velocity_vector[0], vy=-velocity_vector[1], vz=velocity_vector[2])
        xe,ye,ze = get_predicted_point_at_time(params,t)
        points.append([xe,ye,ze])
    x_coordinates, y_coordinates, z_coordinates = zip(*points)

    ax.clear()
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlim3d(-1, 1)
    ax.set_ylim3d(-1, 22)
    ax.set_zlim3d(-1, 1)
    draw_arm(ax,j1,j2,j3,j4,j5)
    ax.plot3D(x_coordinates,y_coordinates,z_coordinates, label="Parabolic Trajectory")  # Replot trajectory
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Parabolic Trajectory")
    ax.legend()
    #ax.view_init(elev=15, azim=-60)
    fig.canvas.draw_idle()  # Redraw the figure

# Create sliders for velocity components
vx_slider = Slider(ax=plt.axes([0.2, 0.0, 0.65, 0.03]), label="Vx", valmin=-10, valmax=10, valinit=velocity_vector[0], valfmt="%0.1f")
vy_slider = Slider(ax=plt.axes([0.2, 0.05, 0.65, 0.03]), label="Vy", valmin=0.1, valmax=40, valinit=velocity_vector[1], valfmt="%0.1f")
vz_slider = Slider(ax=plt.axes([0.2, 0.1, 0.65, 0.03]), label="Vz", valmin=-40, valmax=40, valinit=velocity_vector[2], valfmt="%0.1f")

x0_slider = Slider(ax=plt.axes([0.2, 0.2, 0.65, 0.03]), label="X0", valmin=-1, valmax=1, valinit=x0, valfmt="%0.1f")
z0_slider = Slider(ax=plt.axes([0.2, 0.15, 0.65, 0.03]), label="Z0", valmin=0, valmax=2, valinit=z0, valfmt="%0.1f")
button = Button(ax=plt.axes([0.85, 0.15, 0.1, 0.03]), label="Send")
button.on_clicked(sendMessage)

# Connect sliders to the update function
vx_slider.on_changed(update_trajectory)
vy_slider.on_changed(update_trajectory)
vz_slider.on_changed(update_trajectory)
x0_slider.on_changed(update_trajectory)
z0_slider.on_changed(update_trajectory)


# Plot the initial trajectory
update_trajectory(None)  # Initial plot
draw_arm(ax,j1,j2,j3,j4,j5)

while True:
    plt.pause(0.01)

plt.show()
