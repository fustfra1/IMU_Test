from vpython import *
from time import *
import numpy as np
import math
import serial
# für Plotten: (https://learn.sparkfun.com/tutorials/graph-sensor-data-with-python-and-matplotlib/update-a-graph-in-real-time)
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation


serialPort = serial.Serial(port="COM3", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ""                           # Used to hold data coming over UART

sleep(1)

# STL to vpython funktion: https://vpython.org/presentation2018/FAQ.html


def stl_to_triangles(fileinfo):  # specify file
    # Accept a file name or a file descriptor; make sure mode is 'rb' (read binary)
    fd = open(fileinfo, mode='rb')
    text = fd.read()
    tris = []  # list of triangles to compound
    if False:  # prevent executing code for binary file
        pass
    else:
        fd.seek(0)
        fList = fd.readlines()

        # Decompose list into vertex positions and normals
        vs = []
        for line in fList:
            FileLine = line.split()
            if FileLine[0] == b'facet':
                N = vec(float(FileLine[2]), float(
                    FileLine[3]), float(FileLine[4]))
            elif FileLine[0] == b'vertex':
                vs.append(vertex(pos=vec(float(FileLine[1]), float(
                    FileLine[2]), float(FileLine[3])), normal=N, color=color.white))
                if len(vs) == 3:
                    tris.append(triangle(vs=vs))
                    vs = []

    return compound(tris)
    # bsp: man = stl_to_triangles('STLbot.stl')


scene.range = 5
toRad = 2*np.pi/360
toDeg = 1/toRad
scene.forward = vector(-1, -1, -1)

scene.width = 600
scene.height = 600

xarrow = arrow(lenght=2, shaftwidth=.1, color=color.red, axis=vector(1, 0, 0))
yarrow = arrow(lenght=2, shaftwidth=.1,
               color=color.green, axis=vector(0, 1, 0))
zarrow = arrow(lenght=4, shaftwidth=.1, color=color.blue, axis=vector(0, 0, 1))

frontArrow = arrow(length=4, shaftwidth=.1,
                   color=color.purple, axis=vector(1, 0, 0))
upArrow = arrow(length=1, shaftwidth=.1,
                color=color.magenta, axis=vector(0, 1, 0))
sideArrow = arrow(length=2, shaftwidth=.1,
                  color=color.orange, axis=vector(0, 0, 1))

# bBoard = box(length=6, width=2, height=.2, opacity=.8, pos=vector(0, 0, 0,))
# bn = box(length=1, width=.75, height=.1,
#          pos=vector(-.5, .1+.05, 0), color=color.blue)
# nano = box(lenght=1.75, width=.6, height=.1,
#            pos=vector(-2, .1+.05, 0), color=color.green)
# myObj = compound([bBoard, bn, nano])
myObj = stl_to_triangles('RCO_Logo_3D.stl')
# myObj.size *= 200
myObj.pos = vec(-20, 0, 0)

# ---Plotten---
# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

# This function is called periodically from FuncAnimation


def animate(i, xs, ys):

    # Read data out of the buffer until a carraige return / new line is found
    serialString = serialPort.readline()
    serialString = str(serialString, 'utf-8')
    splitPacket = serialString.split(",")

    # Werte lesen
    accx = float(splitPacket[0])
    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(accx)

    # Limit x and y lists to 20 items
    xs = xs[-40:]
    ys = ys[-40:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Accelerometer X')
    plt.ylabel('Acceleration mg')


# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(
    fig, animate, fargs=(xs, ys), interval=100)
plt.show()
