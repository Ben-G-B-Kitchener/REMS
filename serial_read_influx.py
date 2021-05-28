#!/usr/bin/env python
import signal
import sys
import time
import serial
#from datetime import datetime
import datetime
from influxdb import DataFrameClient
from influxdb import InfluxDBClient
import pandas as pd

f = open('/media/rems/REMS/rems.txt', 'a')

#client = DataFrameClient(host='localhost', port=8086, database='REMS', username='ben', password='********')

def signal_handler(signal, frame):
	print("bye")
	f.closed
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

ser = serial.Serial(
	port='/dev/ttyUSB0',
	baudrate=115200,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1,
)

while 1:
    x=ser.readline()
    if len(x)>0:
        today = datetime.date.today()
        now = datetime.datetime.now()
        f.write(str(now) + "\t" + x.decode())
        if x is not None:
        	node_id, status, voltage, atmega_temperature, wakeup_time, temperature, humidity, rssi = x.split()
        	node_id = int(node_id)
        	node_id = str(node_id)
        	print(node_id)
        
        	raw_data = {}
        	raw_data['node_id'] = int(node_id)
        	raw_data['status'] = int(status)
        	raw_data['voltage'] = float(voltage)
        	raw_data['atmega_temperature'] = int(atmega_temperature)
        	raw_data['wakeup_time'] = int(wakeup_time)
        	raw_data['temperature'] = float(temperature)
        	raw_data['humidity'] = float(humidity)
        	raw_data['rssi'] = int(rssi)
         
        	myTags = {
                	"unit_id": str(99),
                	"location": "none",
                	"sensor": "none"
                	}        
	                
        	if "2" in node_id:
            		myTags = {
                	"unit_id": str(node_id),
                	"location": "LOCATION 1",
                	"sensor": "SHT85"
                	}        

        	if "3" in node_id:
            		myTags = {
                	"unit_id": str(node_id),
                	"location": "LOCATION 2",
                	"sensor": "SHT85"
                	}        

 
        	df = pd.DataFrame(data=raw_data, index=[pd.Timestamp.utcnow()])
        	#res = client.write_points(df, 'data', myTags, protocol='line')
        	print (df)
        	print (myTags)

