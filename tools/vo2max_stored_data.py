# Simple example python to receive, plot and save as csv
# the stored values from the vo2max mask buffer
# sent over WiFi via UDP 
# when user selects the Send stored menu item

# Original UDP plotting code from
# https://github.com/damianjwilliams/udpplotscroll
# Modified 11/2024 L. Peltonen

# Copyright 2024 L. Peltonen
# MIT license
# https://opensource.org/license/MIT


import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
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

samples = 1024      # Max number of samples
slowUpdateRate = 1.0   # Update the "slow" values every 1 sec

record = True       # Store data also as csv file


stored_packet = 0x02  # Status byte indicating that this is a stored packet
data_format_header = "<BBH"
data_format_data = "<ffffffff"
data_format_conf = "<HHH"
headers = ["Time [s]", "Sample time [s]", "VO2 [ml/min/kg]", "Ve [l/min]", "VCO2 [ml/min/kg]", "Freq [1/min]", "HR [bpm]", "Temperature [degC]", "Pressure [hPa]", "O2 [%]"]
columns = {'vo2':0,
           've':1,
           'vco2':2,
           'freq':3,
           'hr':4,
           'temperature':5,
           'pressure':6,
           'o2':7}


# Capture and parse ESP32 data from UDP socket
# in a raw binary format
host = ''   # Listen on default (or all?) interfaces
port = 8008
data_size = struct.calcsize(data_format_data)
header_size = struct.calcsize(data_format_header)
conf_size = struct.calcsize(data_format_conf)
sock_data = None
sock_data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_data.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_data.bind((host, port))
#sock_data.setblocking(False)
#sock_data.settimeout(0.1)   # 100 ms timeout

# Array to store all the chunks of data --> memory content
storage = [0]*samples
received_all = False
largest_chunk = 0
write_position = 0
store_data_rate = 0
calculation_rate = 0


outfile = None


# Parse binary data to values
def socketparse():
    global received_all
    global largest_chunk
    global storage
    global write_position
    global store_data_rate
    global calculation_rate

    
    r_data, _, _ = select.select([sock_data], [], [], 0)
    if not r_data:
        return None
    
    data_packet = sock_data.recv(1500) # Max packet to receive
    #print('Received data: ', len(data_packet))
    
    if len(data_packet) < header_size:
        print('Header not received completely: ', len(data_packet))
        return None
    
    header_data = struct.unpack(data_format_header, data_packet[0:header_size])
    if header_data[0] != stored_packet:
        return None     # Not stored, but maybe realtime packet -> ignore
    
    chunk_num = header_data[1]  # Chunk number
    data_part_size = header_data[2]  # How many bytes should follow the header
    
    if data_part_size + header_size > len(data_packet):
        print('Too few bytes received {}/{}'.format(len(data_packet), data_part_size+header_size))
        return None
    
    if chunk_num > 0:
        # Chunks with num > 0 are actual stored data
        if received_all == True:
            # Got a chunk from new set of data --> clear previous ones
            storage = []
            largest_chunk = chunk_num   # Starts always with last chunk
            received_all = False
        
        n_values = int(data_part_size / data_size)
        #print("Got", n_values, "values")
        
        for n in range(n_values):
            dataset_num = (largest_chunk - chunk_num) * n_values + n
            pos = n * data_size + header_size
            values = struct.unpack(data_format_data, data_packet[pos:pos+data_size])
            #print(values)
            storage[dataset_num] = values
            
    else:
        # Chunk number 0 indicates end of data and configuration values
        #print('Last chunk size {}'.format(len(data_packet)))
        values = struct.unpack(data_format_conf, data_packet[header_size:])
        #print(values)
        write_position = values[0]
        store_data_rate = values[1]
        calculation_rate = values[2]
        received_all = True
        print("Received a stored data package!")
        pass

    #return values
    return None



# Create empty arrays
dataVO2 = deque([0]*samples, maxlen=samples)        # Slow
dataVE = deque([0]*samples, maxlen=samples)         # Slow
dataVCO2 = deque([0]*samples, maxlen=samples)        # Slow
dataRespRate = deque([0]*samples, maxlen=samples)       # Fast update
dataHR = deque([0]*samples, maxlen=samples)         # Slow update

fontsize = '20pt'

# Create plots
win = pg.GraphicsLayoutWidget()
win.setWindowTitle('Vo2Max memory loader')


win.addLabel(text='VO2[ l/min/kg]', row=1, col=0, colspan=1)
p1 = win.addPlot(row=2, col=0, rowspan=1, colspan=1)
p1.setYRange(0, 80, padding=0)

win.addLabel(text='VE [l/min]', row=3, col=0, colspan=1)
p2 = win.addPlot(row=4, col=0, rowspan=1, colspan=1)
p2.setYRange(0, 150, padding=0)

win.addLabel(text='VCO2 [l/min/kg]', row=5, col=0, colspan=1)
p3 = win.addPlot(row=6, col=0, rowspan=1, colspan=8)
p3.setYRange(0, 80, padding=0)

win.addLabel(text='Respiratory rate [1/min]', row=7, col=0, colspan=1)
p4 = win.addPlot(row=8, col=0, rowspan=1, colspan=1)
p4.setYRange(0, 30, padding=0)

win.addLabel(text='Heart Rate [bpm]', row=9, col=0, colspan=1)
p5 = win.addPlot(row=10, col=0, rowspan=1, colspan=1)
p5.setYRange(0, 200, padding=0)



#Create lines for plot
curve1 = p1.plot(dataVO2, pen=pg.mkPen('g', width=2))
curve2 = p2.plot(dataVE, pen=pg.mkPen('r', width=2))
curve3 = p3.plot(dataVCO2, pen=pg.mkPen('b', width=2))
curve4 = p4.plot(dataRespRate, pen=pg.mkPen('c', width=2))
curve5 = p5.plot(dataHR, pen=pg.mkPen('y', width=2))



# Update lists of measurments and add to plots
def updatelist(data):
    global dataflow, dataHR, dataVO2, dataVE
    global curve1, curve2, curve3, curve4
    global storage
    global write_position

    dataVO2.clear()
    dataVE.clear()
    dataVCO2.clear()
    dataRespRate.clear()
    dataHR.clear()

    index = write_position  # Write position indicates the next cell to update --> oldest sample
    max_idx = len(storage)
    for i in range(max_idx):
        if index >= max_idx:
            index = 0
        
        data = storage[index]
        index += 1

        # VO2 plot        
        dataVO2.append(data[columns['vo2']])
        # VE plot
        dataVE.append(data[columns['ve']])
        # VCO2 plot
        dataVCO2.append(data[columns['vco2']])
        # VCO2 plot
        dataRespRate.append(data[columns['freq']])
        # HR plot
        dataHR.append(data[columns['hr']])
    

    curve1.setData(dataVO2)
    curve2.setData(dataVE)
    curve3.setData(dataVCO2)
    curve4.setData(dataRespRate)
    curve5.setData(dataHR)


# update all plots
def updateplot():
    global outfile
    global received_all
    global storage
    global write_position
    global store_data_rate
    global calculation_rate
    
    done_already = received_all

    data = socketparse()
    if not data and not received_all:
        return
    
    
    # This was not last packet
    if not received_all or done_already:
        return
    
    #print("All received")
    received_all = False # Ready for next transmit
    
    # Last packet --> update graphs
    updatelist(data)
    
    
    # Write data to file
    filename = datetime.now().strftime("%Y_%m_%d_%H_%M_%S_vo2_stored.csv")
    if record:
        outfile = open(filename, "w")
        if not outfile:
            return

        # Write headers to CSV file
        outfile.write(','.join(str(x) for x in headers))
        outfile.write('\n')

        t_now = time.time() # Time when the data was received --> assume the "latest" storage time
        sample_rate = (store_data_rate+1) * (calculation_rate/1000)

        index = write_position  # Write position indicates the next cell to update --> oldest sample
        max_idx = len(storage)
        sample_time = -max_idx * sample_rate   # This is the time of the oldest sample
        for i in range(max_idx):
            if index >= max_idx:
                index = 0
            
            data = storage[index]
            index += 1
            
            sample_time += sample_rate
            
            outfile.write(str(t_now+sample_time))
            outfile.write(',')
            outfile.write(str(sample_time))
            outfile.write(',')
            outfile.write(','.join(str(x) for x in data))
            outfile.write('\n')

        outfile.close()


timer = pg.QtCore.QTimer()
timer.timeout.connect(updateplot)
timer.start(10)

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys

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