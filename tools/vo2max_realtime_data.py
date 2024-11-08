# Simple example to receive, plot and store as csv
# the realtime data sent by the vo2max mask over Wifi & UDP

# Original UDP plotting code from
# https://github.com/damianjwilliams/udpplotscroll
# Modified 2/2024 L. Peltonen

# Copyright 2024 L. Peltonen
# MIT license
# https://opensource.org/license/MIT


import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
import socket
import select
try:
    import thread
except ImportError:
    import _thread as thread

from collections import deque
import struct
from datetime import datetime
import time

samples = 300
slowUpdateRate = 1.0   # Update the "slow" values every 1 sec

record = True       # Record data also as csv file


realtime_packet = 0x01  # Status byte indicating that this is a realtime packet
data_format = "<BBHfffffffffff"
headers = ["Time [s]", "Elapsed time [s]", "Flow [Pa]", "O2 [%]", "Ve [l/min]", "VO2 [l/min/kg]", "VCO2 [l/min/kg]", "Freq [1/min]", "P [hPa]", "T [C]", "Texh [C]", "HR [bpm]", "RR [ms]", "Errors"]
columns = {'status':0,
           'dummy':1,
           'err':2,
           'flow':3,
           'o2':4,
           've':5,
           'vo2':6,
           'vco2':7,
           'freq':8,
           'p':9,
           't':10,
           'tex':11,
           'hr':12,
           'rr':13}


# Capture and parse ESP32 data from UDP socket
# in a raw binary format
host = ''   # Listen on default (or all?) interfaces
port = 8008
buffer_size = struct.calcsize(data_format)
sock_data = None
sock_data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_data.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_data.bind((host, port))
#sock_data.setblocking(False)
#sock_data.settimeout(0.1)   # 100 ms timeout

print('Listening on port {:d}'.format(port))
print('Expecting packets of size {:d}'.format(buffer_size))

outfile = None
filename = datetime.now().strftime("%Y_%m_%d_%H_%M_%S_vo2_record.csv")
if record:
    outfile = open(filename, "w")

    # Write headers to CSV file    
    outfile.write(','.join(str(x) for x in headers))
    outfile.write('\n')

start_time = None # Hold the first timestamp so we also get relative time

# Parse binary data to values
def socketparse():    
    r_data, _, _ = select.select([sock_data], [], [], 0)
    if not r_data:
        return None
    
    raw_data = sock_data.recv(buffer_size)
    if len(raw_data) != buffer_size:
        print(len(raw_data))
        return None
    
    values = struct.unpack(data_format, raw_data)
    if values[columns['status']] != realtime_packet:
        return None

    return values


# Create empty arrays
dataFlow = deque([0]*samples, maxlen=samples)       # Fast update
dataHR = deque([0]*samples, maxlen=samples)         # Slow update
dataVO2 = deque([0]*samples, maxlen=samples)        # Slow
dataVE = deque([0]*samples, maxlen=samples)         # Slow

fontsize = '20pt'

# Create plots
win = pg.GraphicsLayoutWidget()
win.setWindowTitle('Vo2Max monitor')

win.addLabel(text='Heart rate [bpm]', row=0, col=0)
win.addLabel(text='VO2 [ml/min/kg]', row=0, col=1)
win.addLabel(text='VE [l/min]', row=0, col=2)
win.addLabel(text='Freq [1/min]', row=0, col=3)
win.addLabel(text='O2 [%]', row=0, col=4)
win.addLabel(text='Pressure [hPa]', row=0, col=5)
win.addLabel(text='Temperature [C]', row=0, col=6)
win.addLabel(text='T exhale [C]', row=0, col=7)

value1 = win.addLabel(text='0', row=1, col=0, size=fontsize)
value2 = win.addLabel(text='0', row=1, col=1, size=fontsize)
value3 = win.addLabel(text='0', row=1, col=2, size=fontsize)
value4 = win.addLabel(text='0', row=1, col=3, size=fontsize)
value5 = win.addLabel(text='0', row=1, col=4, size=fontsize)
value6 = win.addLabel(text='0', row=1, col=5, size=fontsize)
value7 = win.addLabel(text='0', row=1, col=6, size=fontsize)
value8 = win.addLabel(text='0', row=1, col=7, size=fontsize)



win.addLabel(text='Flow [Pa]', row=3, col=0, colspan=8)
p1 = win.addPlot(row=4, col=0, rowspan=1, colspan=8)
p1.setYRange(-10, 250, padding=0)

win.addLabel(text='Heart Rate [bpm]', row=5, col=0, colspan=8)
p2 = win.addPlot(row=6, col=0, rowspan=1, colspan=8)
p2.setYRange(0, 200, padding=0)

win.addLabel(text='VO2 [l/min/kg]', row=7, col=0, colspan=8)
p3 = win.addPlot(row=8, col=0, rowspan=1, colspan=8)
p3.setYRange(0, 80, padding=0)

win.addLabel(text='VE [l/min]', row=9, col=0, colspan=8)
p4 = win.addPlot(row=10, col=0, rowspan=1, colspan=8)
p4.setYRange(0, 150, padding=0)




#Create lines for plot
curve1 = p1.plot(dataFlow, pen=pg.mkPen('g', width=2))
curve2 = p2.plot(dataHR, pen=pg.mkPen('r', width=2))
curve3 = p3.plot(dataVO2, pen=pg.mkPen('b', width=2))
curve4 = p4.plot(dataVE, pen=pg.mkPen('c', width=2))

lastUpdateTime = 0

# Update lists of measurments and add to plots
def updatelist(data):
    global dataflow, dataHR, dataVO2, dataVE
    global curve1, curve2, curve3, curve4
    global lastUpdateTime, slowUpdateRate

    # Flow plot
    dataFlow.append(data[columns['flow']])
    curve1.setData(dataFlow)

    t = time.time()
    if (t-lastUpdateTime) >= slowUpdateRate:
        lastUpdateTime = t
        
        # HR plot
        dataHR.append(data[columns['hr']])
        curve2.setData(dataHR)

        # VO2 plot        
        dataVO2.append(data[columns['vo2']])
        curve3.setData(dataVO2)
    
        # VE plot
        dataVE.append(data[columns['ve']])
        curve4.setData(dataVE)


# update all plots
def updateplot():
    global outfile, start_time
    data = socketparse()
    if not data:
        return

    updatelist(data)
    value1.setText('{:.0f}'.format(data[columns['hr']]))
    value2.setText('{:.2f}'.format(data[columns['vo2']]))
    value3.setText('{:.2f}'.format(data[columns['ve']]))
    value4.setText('{:.0f}'.format(data[columns['freq']]))
    value5.setText('{:.2f}'.format(data[columns['o2']]))
    value6.setText('{:.1f}'.format(data[columns['p']]))
    value7.setText('{:.1f}'.format(data[columns['t']]))
    value8.setText('{:.1f}'.format(data[columns['tex']]))
    
    if outfile:
        # print(','.join(str(x) for x in data))
        t = time.time()
        if not start_time:
            start_time = t

        outfile.write(str(t))
        outfile.write(',')
        outfile.write(str(t-start_time))
        outfile.write(',')
        outfile.write(','.join(str(x) for x in data))
        outfile.write('\n')

timer = pg.QtCore.QTimer()
timer.timeout.connect(updateplot)
timer.start(10)

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys

    #if True or (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        #QtWidgets.QApplication.instance().exec_()
        
    app = QtWidgets.QApplication([])
    win.show()
    app.exec_()
    timer.stop()
    app.quit()
    
    if sock_data:
        sock_data.close()
    
    if outfile:
        outfile.close()
    
    sys.exit()