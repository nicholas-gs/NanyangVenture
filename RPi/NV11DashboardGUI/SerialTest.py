import serial
import time
from DataPoint import NV11DataAccessories, NV11DataSpeedo

s = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
dataAcc = NV11DataAccessories(0x10)
dataSpeed = NV11DataSpeedo(0x0A)
serialListeners = [dataAcc, dataSpeed]
while True:
	line = s.readline().decode()
	for dataPoint in serialListeners:
		if dataPoint.checkMatchString(line):
			dataPoint.unpackString(line) 
			# unpackString() will then debug-print from inside
	print('-',end='')
	time.sleep(0.01)
