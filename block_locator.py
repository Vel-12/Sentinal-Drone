#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
from sensor_msgs.msg import CameraInfo,Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64MultiArray
from sentinel_drone.msg import Geolocation
from osgeo import gdal
import PIL.Image
import cv2
import rospy
import numpy as np
import time
import image_geometry
import imutils


class Edrone():
	
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	# roll, pitch, throttle

		# [x_setpoint, y_setpoint, z_setpoint]
		self.waypoint = [[0,0,18],[-5,6,18],[5,-5,18],[-7,-4,18],[7,1,18],[2,-2,18],[2,0,18],[0,0,18]]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.setpoint = self.waypoint[0]

		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [6,7,10] #6  4.2  10
		self.Ki = [0,0,0]
		self.Kd = [334,334,200] #334  334  200


		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.prev_values = [0,0,0]
		self.max_values = 1800
		self.min_values = 1200
		self.error = [0,0,0]
		self.errsum=[0,0,0]
		self.derr=[0,0,0]
		self.counter = 0
		self.pos = [0,0,0]
		self.flag = 1
		self.objcounter = 0
		
		self.bridge = CvBridge()
		self.data_to_send = Float64MultiArray() 
		self.cam_info = rospy.wait_for_message("/edrone/camera_rgb/camera_info", CameraInfo, timeout=None)
		self.img_proc = image_geometry.PinholeCameraModel()
		self.img_proc.fromCameraInfo(self.cam_info)
		self.ds = gdal.Open(r'/home/tesla/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/updated.tif')

		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.060 # in seconds



		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.altError = rospy.Publisher('/alt_error',Float64, queue_size=1)
		self.pitchError = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.rollError = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.imgpub = rospy.Publisher("/drone_capture_yellowbox",Image,queue_size=10)
		self.yellowcenterpixelpub = rospy.Publisher("/yellow_center_position",Float64MultiArray, queue_size=10)
		self.latlongpub = rospy.Publisher('/geolocation',Geolocation,queue_size=10)

		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber("/edrone/camera_rgb/image_raw",Image, self.image_callback)
		rospy.Subscriber("/yellow_center_position",Float64MultiArray, self.getcenter)
		rospy.Subscriber("/drone_capture_yellowbox",Image, self.pixeloflatlong)

		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE



	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1100
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		
		self.drone_position[0] = msg.poses[0].position.x
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

		#---------------------------------------------------------------------------------------------------------------

	
	def image_callback(self,img_msg):
		try:
			dst = self.bridge.imgmsg_to_cv2(img_msg,"passthrough")
			img = cv2.cvtColor(dst,cv2.COLOR_BGR2RGB) 
		except CvBridgeError as e:
			rospy.logger("CvBridge Error: {0}".format(e))

		hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		lower = np.array([22, 93, 0])
		upper = np.array([45, 255, 255])

		mask = cv2.inRange(hsv,lower,upper)

		cnts = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)

		M={}
		for c in cnts:
			area = cv2.contourArea(c)
			if(area>1800):
				cv2.drawContours(img,[c],-1,(0,255,0),2)
				M = cv2.moments(c)
				cx = M["m10"] / M["m00"]
				cy = M["m01"] / M["m00"]
				self.center = list() 
				self.center.append(cx)
				self.center.append(cy)
				self.data_to_send.data = self.center
				if self.flag == 1:
					self.imgpub.publish(img_msg)
					self.yellowcenterpixelpub.publish(self.data_to_send)
					self.objcounter += 1
					self.flag = 0
		if(not bool(M)):
			self.flag = 1
		# cv2.imshow("Image Window",img)
		# cv2.waitKey(1)

	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		# This is just for an example. You can change the ratio/fraction value accordingly
		self.Kp[2] = alt.Kp * 0.01
		self.Ki[2] = alt.Ki * 0.001
		self.Kd[2] = alt.Kd * 0.1
	
		
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = pitch.Ki * 0.008
		self.Kd[1] = pitch.Kd * 0.3


	def roll_set_pid(self,roll):
		self.Kp[1] = roll.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = roll.Ki * 0.008
		self.Kd[1] = roll.Kd * 0.3

	def changeWaypoint (self,error):
		if(abs(error[2])<=0.2):
			if(abs(error[1])<=0.2):
				if(abs(error[0])<=0.2):
					self.counter+=1
					if(self.counter <= 7):
						self.setpoint = self.waypoint[self.counter]
	

	def pixel2coord(self,x, y):
		"""Returns global coordinates from pixel x, y coords"""
		xoff, a, b, yoff, d, e = self.ds.GetGeoTransform()
		xp = a * x + b * y + xoff
		yp = d * x + e * y + yoff
		return xp, yp

	def getcenter(self,data):
		self.centerpoint = list()
		self.centerpoint = data.data

	def pixeloflatlong(self,img_msg_data):
		MIN_MATCH_COUNT = 10
		try:
			dst = self.bridge.imgmsg_to_cv2(img_msg_data,"passthrough")
			img1 = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
		except CvBridgeError as e:
			rospy.logger("CvBridge Error: {0}".format(e))
		img=PIL.Image.open(r'/home/tesla/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/updated.tif')
		img3 = np.array(img)
		img2 = cv2.cvtColor(img3, cv2.COLOR_BGR2GRAY)
		# Initiate SIFT detector
		sift = cv2.SIFT_create()
		# find the keypoints and descriptors with SIFT
		kp1, des1 = sift.detectAndCompute(img1,None)
		kp2, des2 = sift.detectAndCompute(img2,None)

		kps_img = cv2.drawKeypoints(image= img2,keypoints=kp2,outImage=img2)


		FLANN_INDEX_KDTREE = 1
		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		search_params = dict(checks = 50)
		flann = cv2.FlannBasedMatcher(index_params, search_params)
		matches = flann.knnMatch(des1,des2,k=2)
		# store all the good matches as per Lowe's ratio test.
		good = []
		for m,n in matches:
			if m.distance < 0.75*n.distance:
				good.append(m)

		if len(good)>MIN_MATCH_COUNT:
			src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
			dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
			M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
			matchesMask = mask.ravel().tolist()
			h,w = img1.shape
			pts = np.float32( [0+self.centerpoint[0],0+self.centerpoint[1]]).reshape(-1,1,2)
			dst = cv2.perspectiveTransform(pts,M)
			lati,longi = self.pixel2coord(dst[0][0][0],dst[0][0][1])
			self.latlongpub.publish(str(self.objcounter),float(lati),float(longi))

			# img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
			# cv2.imshow("test",img1)
			# cv2.waitKey(0)
			# cv2.destroyAllWindows()

	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum


		#Getting error of all coordinates		
		self.error[0]= self.setpoint[0] - self.drone_position[0]    # roll
		self.error[1]=  self.drone_position[1] - self.setpoint[1]	# pitch
		self.error[2]= -(self.setpoint[2] - self.drone_position[2]) # throttle



		#Integration for Ki
		#self.errsum[0]=self.errsum[0]+(self.error[0])
		#self.errsum[1]=self.errsum[1]+(self.error[1])
		self.errsum[2]+=(self.error[2])


		#Derivation for Kd
		self.derr[0]=(self.error[0]-self.prev_values[0])
		self.derr[1]=(self.error[1]-self.prev_values[1])
		self.derr[2]=(self.error[2]-self.prev_values[2])


		#Calculating output in 1500
		self.cmd.rcRoll=int(1500+(self.Kp[0]*self.error[0])+(self.Kd[0]*self.derr[0]))
		self.cmd.rcPitch=int(1500+(self.Kp[1]*self.error[1])+(self.Kd[1]*self.derr[1]))
		self.cmd.rcThrottle=int(1500 + self.Kp[2]*self.error[2] + self.Ki[2] * self.errsum[2] + self.Kd[2]*self.derr[2])

				
		#Checking min and max threshold and updating on true
			#Throttle Conditions
		if self.cmd.rcThrottle>1800:
			self.cmd.rcThrottle=self.max_values
		if self.cmd.rcThrottle<1200:
			self.cmd.rcThrottle=self.min_values	


			#Pitch Conditions
		if self.cmd.rcPitch>1800:
			self.cmd.rcPitch=self.max_values	
		if self.cmd.rcPitch<1200:
			self.cmd.rcPitch=self.min_values


			#Roll Conditions
		if self.cmd.rcRoll>1800:
			self.cmd.rcRoll=self.max_values
		if self.cmd.rcRoll<1200:
			self.cmd.rcRoll=self.min_values


		#Publishing values on topic 'drone command'
		self.command_pub.publish(self.cmd)
				
		#Updating prev values for all axis
		self.prev_values[0]=self.error[0]
		self.prev_values[1]=self.error[1]
		self.prev_values[2]=self.error[2]
		
		self.changeWaypoint(self.error)

		#Getting values for Plotjuggler
		self.rollError.publish(self.error[0])
		self.pitchError.publish(self.error[1])
		self.altError.publish(self.error[2])


	#------------------------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
	e_drone = Edrone()
	r = rospy.Rate(15) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	time.sleep(5)
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()