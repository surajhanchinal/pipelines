import PySimpleGUI as sg
import matplotlib.pyplot as plt
from ik_helpers import update_trajectory
import redis
import numpy as np

# Connect to the Redis server
redis_client = redis.StrictRedis(host='localhost', port=6379, db=0)

# Create a pubsub object
pubsub = redis_client.pubsub()

redis_client2 = redis.StrictRedis(host='localhost', port=6379, db=0)

def sendMessage():
    global velocity_vector,x0,y0,z0
    redis_client2.publish('traj-channel',"{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}".format(vx,vy,vz,x0,y0,z0))

sols = []
selectedIndex = 0

def handler(message):
    global j1,j2,j3,j4,j5
    global sols,selectedIndex
    global window
    sols = []
    print(message['data'].decode())
    solutions = message['data'].decode().split('|')
    for sol in solutions:
        angles = sol.split()[1:]
        angles = [float(a) for a in angles]
        j1 = angles[0]*(np.pi/2000)
        j2 = angles[1]*(np.pi/1800) 
        j3 = angles[2]*(np.pi/1800) + np.pi/2
        j4 = angles[3]*(np.pi/1800)
        j5 = angles[4]*(np.pi/1800)
        sols.append([j1,j2,j3,j4,j5])
    selectedIndex = 1
    updateAngles()
pubsub.subscribe(**{'solution': handler})
pubsub.run_in_thread(sleep_time=0.01)


x0 =  1.59768
y0 = 15.4951
z0 =  1.81544
vx = -1.82261
vy = -17.2788
vz = -1.905
j1 = 0
j2 = 0
j3 = 0
j4 = 0
j5 = 0

def updateAngles():
    global j1,j2,j3,j4,j5
    global window
    if(selectedIndex > 0):
        [j1,j2,j3,j4,j5] = sols[selectedIndex-1]
        window['selected-sol'].update(str(selectedIndex)+'/'+str(len(sols)))

fig, ax = plt.subplots(figsize=(8, 6), subplot_kw={"projection": "3d"})
def draw_plot():
    global x0,y0,z0,vx,vy,vz
    global j1,j2,j3,j4,j5
    update_trajectory(ax,fig,vx,vy,vz,x0,y0,z0,j1,j2,j3,j4,j5)
    plt.pause(0.008)

layout = [
          [sg.Text('X '),sg.Slider(range=(-1000,1000),default_value=int(x0*50),expand_x=True,enable_events=True,orientation='horizontal',key='x-sl',disable_number_display=True,),sg.Text(str(x0),key='x-sl-val')],
          [sg.Text('Y '),sg.Slider(range=(-1000,1000),default_value=int(y0*50),expand_x=True,enable_events=True,orientation='horizontal',key='y-sl',disable_number_display=True,),sg.Text(str(y0),key='y-sl-val')],
          [sg.Text('Z '),sg.Slider(range=(-1000,1000),default_value=int(z0*50),expand_x=True,enable_events=True,orientation='horizontal',key='z-sl',disable_number_display=True,),sg.Text(str(z0),key='z-sl-val')],
          [sg.Text('VX'),sg.Slider(range=(-1000,1000),default_value=int(vx*50),expand_x=True,enable_events=True,orientation='horizontal',key='vx-sl',disable_number_display=True,),sg.Text(str(vx),key='vx-sl-val')],
          [sg.Text('VY'),sg.Slider(range=(-1000,1000),default_value=int(-vy*50),expand_x=True,enable_events=True,orientation='horizontal',key='vy-sl',disable_number_display=True,),sg.Text(str(vy),key='vy-sl-val')],
          [sg.Text('VZ'),sg.Slider(range=(-1000,1000),default_value=int(vz*50),expand_x=True,enable_events=True,orientation='horizontal',key='vz-sl',disable_number_display=True,),sg.Text(str(vz),key='vz-sl-val')],
          [sg.Button('Find solution',key='fs')],
          [sg.Button('Prev',key='prev'),sg.Button('Next',key='next'),sg.Text('0/0',key='selected-sol')]]

window = sg.Window('Have some Matplotlib....', layout,size=(500,500))

while True:
    event, values = window.read(timeout=8)
    draw_plot()
    if event in (sg.WIN_CLOSED, 'Cancel'):
        break
    elif event == 'x-sl':
        x0 = values['x-sl']/50
        window['x-sl-val'].update(str(x0))
    elif event == 'y-sl':
        y0 = values['y-sl']/50
        window['y-sl-val'].update(str(y0))
    elif event == 'z-sl':
        z0 = values['z-sl']/50
        window['z-sl-val'].update(str(z0))
    elif event == 'vx-sl':
        vx = values['vx-sl']/50
        window['vx-sl-val'].update(str(vx))
    elif event == 'vy-sl':
        vy = -values['vy-sl']/50
        window['vy-sl-val'].update(str(vy))
    elif event == 'vz-sl':
        vz = values['vz-sl']/100
        window['vz-sl-val'].update(str(vz))
    elif event == 'fs':
        sendMessage()
    elif event == 'next':
        selectedIndex = min(len(sols),selectedIndex+1)
        updateAngles()
    elif event == 'prev':
        selectedIndex = max(1,selectedIndex-1)
        updateAngles()
window.close()