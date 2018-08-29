import transmitter as tm
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import threading
from ch_motion import *

# Application
app = QtGui.QApplication([])

# Window
win = pg.GraphicsWindow(title='Hands AHRS')
win.resize(1500, 600)
win.setWindowTitle('Hands AHRS: Ploting')

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

# Acc curve
pacc = win.addPlot(title="Acc")

# Gyro curve
pgyro = win.addPlot(title='Gyro')

# Euler curve
peuler = win.addPlot(title='Euler')

# Plot setting
pen_width = 2
fill_beta = 70
pen_red = pg.mkPen('F00', width=pen_width)
pen_green = pg.mkPen('0F0', width=pen_width)
pen_blue = pg.mkPen('0AF', width=pen_width)
fill_red = [255, 0, 0, fill_beta]
fill_green = [0, 255, 0, fill_beta]
fill_blue = [0, 200, 255, fill_beta]

length = 200
w_controller_state = pg.LayoutWidget()
pacc.showGrid(x=True, y=True)
pgyro.showGrid(x=True, y=True)
peuler.showGrid(x=True, y=True)
curve_ax = pacc.plot(pen=pen_red, fillLevel=0, brush=fill_red)
curve_ay = pacc.plot(pen=pen_green, fillLevel=0, brush=fill_green)
curve_az = pacc.plot(pen=pen_blue, fillLevel=0, brush=fill_blue)
curve_gx = pgyro.plot(pen=pen_red, fillLevel=0, brush=fill_red)
curve_gy = pgyro.plot(pen=pen_green, fillLevel=0, brush=fill_green)
curve_gz = pgyro.plot(pen=pen_blue, fillLevel=0, brush=fill_blue)
curve_ex = peuler.plot(pen=pen_red, fillLevel=0, brush=fill_red)
curve_ey = peuler.plot(pen=pen_green, fillLevel=0, brush=fill_green)
curve_ez = peuler.plot(pen=pen_blue, fillLevel=0, brush=fill_blue)

acc_max = 6
gyro_max = 2000
euler_max = 200
text_interval = acc_max*0.08
text_interval_v = acc_max*0.12
acc_text_max_y = acc_max*1.12
gyro_text_max_y = gyro_max*1.12
pacc.setXRange(0, length)
pacc.setYRange(-acc_max, acc_max)
pgyro.setXRange(0, length)
pgyro.setYRange(-gyro_max, gyro_max)
peuler.setXRange(0, length)
peuler.setYRange(-euler_max, euler_max)

# Data
data = np.zeros([9, length])

# AHRS massage
ahrs_msg1 = tm.AHRSMsg("COM5")
hands = Hands()
drum_group = DrumGroup()

# Read data
def read_data():
    global ahrs_msg1, data
    while True:
        ahrs_msg1.read()
        hands.right_hand.set_AHRS(ahrs_msg1)
        acc = hands.right_hand.acc_sensor
        gyro = hands.right_hand.gyro_sensor
        euler = ahrs_msg1.get_euler()
        data = np.roll(data, -1)
        data[0][-1] = acc.x
        data[1][-1] = acc.y
        data[2][-1] = acc.z
        data[3][-1] = gyro.x
        data[4][-1] = gyro.y
        data[5][-1] = gyro.z
        data[6][-1] = euler.x
        data[7][-1] = euler.y
        data[8][-1] = euler.z


# Updating function
def update_plot():
    global curve_ax, curve_ay, curve_az
    global curve_gx, curve_gy, curve_gz
    global curve_ex, curve_ey, curve_ez
    global data
    curve_ax.setData(data[0])
    curve_ay.setData(data[1])
    curve_az.setData(data[2])
    curve_gx.setData(data[3])
    curve_gy.setData(data[4])
    curve_gz.setData(data[5])
    curve_ex.setData(data[6])
    curve_ey.setData(data[7])
    curve_ez.setData(data[8])
timer = QtCore.QTimer()
timer.timeout.connect(update_plot)
timer.start(33)

threads = []
t1 = threading.Thread(target=read_data)
threads.append(t1)

for t in threads:
    t.setDaemon(True)
    t.start()

win.show()
app.exec_()





