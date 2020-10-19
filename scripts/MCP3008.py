#!/usr/bin/env python
# # -*-coding:Latin-1 -*

import spidev
import time

# Define Variables
adc_values = [0,0,0,0,0]
n_loop_mean = 5
n_sensors = 5
delay = 1.0 #in sec
delay_loop = 0.060 #limit is 200k samples/sec for MCP bu sharp is limited by 20/sec

# Create SPI
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1000000
spi.mode =0b00
#spi.cshigh = True

# 5 sensors, ID from 0 to 4 included
# Sensor #0 : Front Left
# Sensor #1 : Front Right
# Sensor #2 : Front Center
# Sensor #3 : Back Left
# Sensor #4 : Back Right

def readadc(sensor_id, mean_n_loops=5):
	if sensor_id > 4 or sensor_id < 0 or mean_n_loops < 1:
		return -1
	ret = 0
	for i in range(mean_n_loops):
		resp = spi.xfer2([0x01, (0x08 + sensor_id) << 4, 0x00])
		ret += ((resp[1] & 0x03) << 8) + resp[2]
		time.sleep(delay_loop)
	return ret/mean_n_loops

def readAlladc():
	for i in range(n_loop_mean):
		for sensor_id in range(n_sensors):
			resp = spi.xfer2([0x01, (0x08 + sensor_id) << 4, 0x00])
			adc_values[sensor_id] += ((resp[1] & 0x03) << 8) + resp[2]
		time.sleep(delay_loop)
	for sensor_id in range(n_sensors):
		adc_values[sensor_id] /= n_loop_mean
	# print(adc_values)
	return adc_values
