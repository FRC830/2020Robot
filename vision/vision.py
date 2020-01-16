#!/usr/bin/env python3
import json
import time
import sys
import numpy as np
import cv2
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink
from networktables import NetworkTablesInstance

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []

"""Report parse error."""
def parseError(str):
	print("config error in '" + configFile + "': " + str, file=sys.stderr)

"""Read single camera configuration."""
def readCameraConfig(config):
	cam = CameraConfig()

	# name
	try:
		cam.name = config["name"]
	except KeyError:
		parseError("could not read camera name")
		return False

	# path
	try:
		cam.path = config["path"]
	except KeyError:
		parseError("camera '{}': could not read path".format(cam.name))
		return False

	# stream properties
	cam.streamConfig = config.get("stream")

	cam.config = config

	cameraConfigs.append(cam)
	return True

"""Read configuration file."""
def readConfig():
	global team
	global server

	# parse file
	try:
		with open(configFile, "rt") as f:
			j = json.load(f)
	except OSError as err:
		print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
		return False

	# top level must be an object
	if not isinstance(j, dict):
		parseError("must be JSON object")
		return False

	# team number
	try:
		team = j["team"]
	except KeyError:
		parseError("could not read team number")
		return False

	# ntmode (optional)
	if "ntmode" in j:
		str = j["ntmode"]
		if str.lower() == "client":
			server = False
		elif str.lower() == "server":
			server = True
		else:
			parseError("could not understand ntmode value '{}'".format(str))

	# cameras
	try:
		cameras = j["cameras"]
	except KeyError:
		parseError("could not read cameras")
		return False
	for camera in cameras:
		if not readCameraConfig(camera):
			return False

	return True

"""Start running the camera."""
def startCamera(config):
	print("Starting camera '{}' on {}".format(config.name, config.path))
	inst = CameraServer.getInstance()
	camera = UsbCamera(config.name, config.path)
	server = inst.startAutomaticCapture(camera=camera, return_server=True)

	camera.setConfigJson(json.dumps(config.config))
	camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

	if config.streamConfig is not None:
		server.setConfigJson(json.dumps(config.streamConfig))

	return camera

if __name__ == "__main__":
	if len(sys.argv) >= 2:
		configFile = sys.argv[1]

	# read configuration
	if not readConfig():
		sys.exit(1)

	# start NetworkTables
	ntinst = NetworkTablesInstance.getDefault()
	dashboard = ntinst.getTable("SmartDashboard")
	# dashboard = table.getSubTable("vision")
	if server:
		print("Setting up NetworkTables server")
		ntinst.startServer()
	else:
		print("Setting up NetworkTables client for team {}".format(team))
		ntinst.startClientTeam(team)

	# start cameras
	cameras = []
	for cameraConfig in cameraConfigs:
		cameras.append(startCamera(cameraConfig))

	inst = CameraServer.getInstance()
	
	height = 120
	width = 160

	videoOutput = inst.putVideo("Rasp PI Output", width, height)
	thresholdOutput = inst.putVideo("Yellow Mask", width, height)
	videoSink = CvSink("Rasp PI Converter")


	img = np.ndarray((height,width,3))
	frontCamera = dashboard.getBoolean("Front Camera", True)
	dashboard.putNumber("Number of Cameras", len(cameras))
	if (frontCamera):
		videoSink.setSource(cameras[0])
	else:
		videoSink.setSource(cameras[1])

	# vision processing
	while not not not False: #True if you can't count
		timestamp, img = videoSink.grabFrame(img) # this outputs a CvImage
		if not timestamp: # could not grab frame
			continue
		img = img.astype(dtype="uint8")
		hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		# read from smartdashboard
		lowerh = dashboard.getNumber("lowerH", 15)
		lowers = dashboard.getNumber("lowerS", 100)
		lowerv = dashboard.getNumber("lowerV", 130)
		upperh = dashboard.getNumber("upperH", 60)
		uppers = dashboard.getNumber("upperS", 255)
		upperv = dashboard.getNumber("upperV", 255)
		dashboard.putNumber("readUpperV", upperv)
		lowerBound = np.array([lowerh, lowers, lowerv])
		upperBound = np.array([upperh, uppers, upperv])
		# get mask of all values that match bounds, then display part 
		# of image that matches bound
		mask = cv2.inRange(hsvImg, lowerBound, upperBound)
		# remove small blobs that may mess up average value
		# https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
		# https://github.com/FRC830/WALL-O/blob/master/vision/vision.py
		# https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/

		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		
		# Find 'parent' contour(s) with simple chain countour algorithm
		contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		# https://github.com/jrosebr1/imutils/blob/master/imutils/convenience.py#L162
		contours = contours[1]
		if len(contours) == 0:
			continue
		
		max_contour = max(contours, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(max_contour) # returns point, radius
		originalRadius = dashboard.getNumber("Original", 7)
		# original radius * distance away / width as described in link #3
		focalLength = (originalRadius * 24.0) / 3.5
		distanceAway = (3.5 * focalLength) / radius

		dashboard.putNumber("Radius", radius)
		dashboard.putNumber("Focal Length", focalLength)
		dashboard.putNumber("Distance Away", distanceAway)
		if radius < dashboard.getNumber("Min Radius", 1):
			continue

		M = cv2.moments(max_contour)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		dashboard.putNumber("centerX", center[0])

		cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
		cv2.circle(img, center, 5, (0, 0, 255), -1)
		videoOutput.putFrame(img)
		maskOut = cv2.bitwise_and(img, img, mask=mask)
		thresholdOutput.putFrame(maskOut)