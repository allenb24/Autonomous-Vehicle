# OpenMV[2] - Stop Sign / Traffic Light Detection

import sensor
import image
import time
import pyb
from pyb import UART

# Intiialize UART port, baud rate to send signals to primary OpenMV
uart = UART(1, 115200)

# Initialize camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)	# RGB format for color detction
sensor.set_framesize(sensor.QVGA)	# 320x240 Resolution
sensor.skip_frames(time=2000)		# Allow camera to warm up

# Set color threshold to detect red (for stop sign)
red_thresholds = [(30, 100, 15, 127, 15, 127)]

# Set blob area for stop sign
min_blob_area = 5500
max_blob_area = 10000
detection_flag = 0

# Helper function to send detection signal over UART
def MVtoMV(ch):
	try:
		uart.write(ch.encode())
		print(ch)
	except KeyboardInterrupt:
		print("Exiting program")
	except Exception as exception_error:
		print("Error occurred. Exiting program")
		print("Error: " + str(exception_error))

while True:
	# Define region of interest
	roi=(120,0,200,300)

	# Capture image and apply rotation correction (camera mounted sideways)
	img = sensor.snapshot()
	img = img.rotation_corr(z_rotation=90)

	# Visualize the roi rectangle on the frame
	a = img.draw_rectangle(roi, color = (0,255,0), thickness=3)

	# Red color detection
	redblobs = img.find_blobs(red_thresholds, roi=roi)
	for blob in redblobs:
		if min_blob_area < blob.area() < max_blob_area:
			if blob.w() / blob.h() > 0.5:	# Filter out oddly shaped blobs
				img.draw_rectangle(blob.rect())
				img.draw_cross(blob.cx(), blob.cy())
				detection_flag = 1
				MVtoMV('R')	# Signal to stop (red)
				print("Possible RED Traffic Signal")
				time.sleep(1)	# Allow time to detect again