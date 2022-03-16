#!/usr/bin/env python


#optimized Video Streaming code.


from __future__ import print_function
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
import rospy
from csi_camera_nano import CSI_Camera
import threading
from std_msgs.msg import String, Bool
import virtualvideo

class Stream_Cameras():
	def __init__(self):
		# Enabling the FPS
		self.show_fps = False	# Change-3: Make it True if you want to see the


		self.virtualDeviceWriter = virtualvideo.FakeVideoDevice()  
		self.virtualDeviceWriter.init_input((1280, 720))             
		self.virtualDeviceWriter.init_output(11, 1280, 720, fps=60)  
		self.virtualDeviceWriter.setup() 


		#Setting up the csi camera width and height
		self.DISPLAY_WIDTH=640
		self.DISPLAY_HEIGHT=480
		self.camera_view = 'front_view'
		# Sensor modes for CSI Cameras
		self.SENSOR_MODE_1080=2
		self.SENSOR_MODE_720=3

		# Initializing the variable for slave frames
		self.rear_left_frame = 0
		self.side_left_frame= 0

		self.rear_right_frame = 0
		self.side_right_frame= 0
	
                #print(len(str(f1)))
		# Initialize the publisher to publish the single frame
		self.pub = rospy.Publisher('camera/debot_cams_broadcaster', Image, queue_size=10)
			
		# Initialize the ROS Node	
		rospy.init_node('debot_cams', anonymous=True)

		# Go through the loop 10 times per second
		self.rate = rospy.Rate(10) # 10hz	# Change-2: Default Rate

		# Initialize the CVBridge to convert the ros msg
		self.br = CvBridge()

		# Parameters for grid lines
		self.gridsfile_binary_l = 'left_grid_points.npy'
		self.gridsfile_binary_r = 'right_grid_points.npy'

		# Saving the grid coordinates to a variable
		with open(self.gridsfile_binary_l, 'rb') as f:
			self.grid_l = np.load(f)
		self.grid_l = self.grid_l*[self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT]

		with open(self.gridsfile_binary_r, 'rb') as f:
			self.grid_r = np.load(f)
		self.grid_r = self.grid_r*[self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT]
		
	# Simple draw label on an image; in our case, the video frame
	def draw_label(self, cv_image, label_text, label_position):
		font_face = cv2.FONT_HERSHEY_SIMPLEX
		scale = 0.5
		color = (255,255,255)
		# You can get the size of the string with cv2.getTextSize here
		cv2.putText(cv_image, label_text, label_position, font_face, scale, color, 1, cv2.LINE_AA)

		# Read a frame from the camera, and draw the FPS on the image if desired
		# Return an image

	def draw_error_status(self, cv_image, label_text, label_position):
		font_face = cv2.FONT_HERSHEY_SIMPLEX
		scale = 0.5
		color = (255,255,255)
		# You can get the size of the string with cv2.getTextSize here
		cv2.putText(cv_image, label_text, label_position, font_face, scale, color, 1, cv2.LINE_AA)

	def read_csi_camera(self, csi_camera,display_fps, cam_id):
		_ , camera_image=csi_camera.read()
		if display_fps:	
			self.draw_label(camera_image, cam_id,(200,20))
		return camera_image

	def cam_view(self, data):
		if data.data == 1:
			if self.camera_view == "front_view":
				self.camera_view = "rear_view"
			elif self.camera_view == "rear_view":
				self.camera_view = "front_view"

	# RR Subscriber callback
	def RearRightCam_callback(self, data):
	 
		# Used to convert between ROS and OpenCV images
		RR_br = CvBridge()

		# Output debugging information to the terminal
		#rospy.loginfo("Recieving RR")

		#global RR_frame
		# Convert ROS Image message to OpenCV image
		self.rear_right_frame = RR_br.imgmsg_to_cv2(data)
                #self.rear_right_frame = cv2.rotate(self.rear_right_frame, cv2.ROTATE_90_CLOCKWISE)

	def RearLeftCam_callback(self, data):
	 
		# Used to convert between ROS and OpenCV images
		RL_br = CvBridge()

		# Output debugging information to the terminal
		#rospy.loginfo("Recieving RR")

		#global RR_frame
		# Convert ROS Image message to OpenCV image
		self.rear_left_frame = RL_br.imgmsg_to_cv2(data)
                #self.rear_left_frame = cv2.rotate(self.rear_left_frame, cv2.ROTATE_90_CLOCKWISE)

	# SL Subscriber callback
	def SideLeftCam_callback(self, data):
	 
		# Used to convert between ROS and OpenCV images
		SL_br = CvBridge()
		 
		# Output debugging information to the terminal
		#rospy.loginfo("Recieving SL")

		#global SL_frame 
		# Convert ROS Image message to OpenCV image
		self.side_left_frame = SL_br.imgmsg_to_cv2(data)

	def SideRightCam_callback(self, data):
	 
		# Used to convert between ROS and OpenCV images
		SR_br = CvBridge()
		 
		# Output debugging information to the terminal
		#rospy.loginfo("Recieving SL")

		#global SL_frame 
		# Convert ROS Image message to OpenCV image
		self.side_right_frame = SR_br.imgmsg_to_cv2(data)


	# Subscribe and recieve the stream from slaves
	def recieve_slave_cameras(self):

		rospy.Subscriber('debot/camera/side_left_cam', Image, self.SideLeftCam_callback)
		rospy.Subscriber('debot/camera/rear_left_cam', Image, self.RearLeftCam_callback)
		rospy.Subscriber('debot/camera/side_right_cam', Image, self.SideRightCam_callback)
		rospy.Subscriber('debot/camera/rear_right_cam', Image, self.RearRightCam_callback)
		rospy.Subscriber('debot/camera/view', Bool, self.cam_view)
		self.rate.sleep()

	# Calculating slope of a line from 2 points
	def slope_intercept(self, pt1, pt2):
		# Computing slope and intercept of line formed by 2 given points
		m = (pt1[1] - pt2[1])/(pt1[0] - pt2[0])
		theta = np.arctan(m)
		c = pt1[1] - m*pt1[0]
		return m, c, theta*180/np.pi

	# Manually correcting the slope to a determined point
	def slope_intercept_symmetry(self, pt1, pt2, dim):
		# Computing slope and intercept of line formed by 2 given points

		m = (pt1[1] - pt2[1])/(pt1[0] - pt2[0])
		c = pt1[1] - m*pt1[0]
		
		# Finding the point at the bottom of the screen
		x_coord = (dim[1] - c)/m
		pt_new = [x_coord, dim[1]]

		return m, c, pt_new

	# Points derived from screen manually to form the calibration lines
	def screen_lines_calibration(self):
		# Scaled corrected points
		screen_width = 20.8               # (cm) Derived manually by measuring with scale on the window
		corrected_points = [18, 14, 9.5, 5.5, 1]    # (cm) Derived manually by measuring the distance of points on the window with a scale from outboard to inboard direction
		corrected_points_scaled = [x/screen_width for x in corrected_points] # Scaling the points to fit any screen
		dist_lines = [4, 4.5, 4 , 4.5]  
		dist_lines_scaled = [x/screen_width for x in dist_lines]  
		return screen_width, corrected_points_scaled, dist_lines_scaled

	# Function to correct the position of bottom points
	def bottom_point_correction(self, index, line, dim):
		# Scaled corrected points
		screen_width, corrected_points_scaled, _ = self.screen_lines_calibration()

		# Choosing the corrected point based on left or right image
		if index == 'l':
			x_coord = dim[0]*(1 - corrected_points_scaled[line])
			pt_corrected = [x_coord, dim[1]]
		else:
			x_coord = dim[0]*(corrected_points_scaled[line])
			pt_corrected = [x_coord, dim[1]]

		return pt_corrected

	def draw_lines_symmetry_point(self, img, grid, dim, index):
		# Initializing parameters
		m_vert = []
		m_hor = []
		colors = [[0,0,255],[0,165,255],[0,255,255],[0,255,0]]
		lane_bottom_points = []
		
		# Setting the width and height fractions to make the point scalable to any window size
		if index == 'l':
			width_fraction = 4.7/18.05
			height_fraction = 7.2/13.6
			vp = (int(dim[0]*(1 - width_fraction)), int(dim[1]* height_fraction))
		else:
			width_fraction = 4.7/18.05
			height_fraction = 7.2/13.6
			vp = (int(dim[0]*width_fraction), int(dim[1]* height_fraction))
		
		# Drawing horizontal and vertical lines
		for i in range(4):
			if i == 0:
				# Drawing horizontal line from x=0 to x=width
				m, c, _ = self.slope_intercept(grid[i][0], grid[i][4])
				m_hor.append([m,c])
				cv2.line(img, (int(0), int(c)), (int(dim[0]), int(dim[0]*m + c)), colors[i], 3)
			else:
				# Drawing horizontal lines from x=0 to x=width
				m, c, _ = self.slope_intercept(grid[i][0], grid[i][6])
				m_hor.append([m,c])
				cv2.line(img, (int(0), int(c)), (int(dim[0]), int(dim[0]*m + c)), colors[i], 3)
				# Drawing vertical lines from y=0 to y=height
				if i == 1:
					for j in range(7):
						m, c, pt_new = self.slope_intercept_symmetry(grid[i][j], grid[i+2][j], dim)
						if j<5:
							pt_new = self.bottom_point_correction(index, j, dim)
							lane_bottom_points.append([int(pt_new[0]), int(pt_new[1])])
						m_vert.append([m, c])
						if j == 3:
							cv2.line(img, vp, (int(pt_new[0]), int(pt_new[1])), [150,50,100], 3)
						else:
							cv2.line(img, vp, (int(pt_new[0]), int(pt_new[1])), [255,255,255], 3)
		
		return img, lane_bottom_points, vp, m_hor
	
	# Start Local Cameras and stitch.
	def start_local_cameras(self):
		
		#Starting CSI Cameras
		self.front_csi_left = CSI_Camera()
		self.front_csi_left.create_gstreamer_pipeline(
			sensor_id=0,
			sensor_mode=self.SENSOR_MODE_720,
			framerate=20,
			flip_method=2,
			display_height=self.DISPLAY_HEIGHT,
			display_width=self.DISPLAY_WIDTH,
		)
		self.front_csi_left.open(self.front_csi_left.gstreamer_pipeline)
		self.front_csi_left.start()

		self.front_csi_right = CSI_Camera()
		self.front_csi_right.create_gstreamer_pipeline(
			sensor_id=1,
			sensor_mode=self.SENSOR_MODE_720,
			framerate=20,
			flip_method=2,
			display_height=self.DISPLAY_HEIGHT,
			display_width=self.DISPLAY_WIDTH,
		)
		self.front_csi_right.open(self.front_csi_right.gstreamer_pipeline)
		self.front_csi_right.start()
		
		if ( not self.front_csi_left.video_capture.isOpened() or not self.front_csi_right.video_capture.isOpened()):

			# Cameras did not open, or no camera attached
			print("Unable to open any cameras")

		try:
			# Start counting the number of frames read and displayed
			self.front_csi_left.start_counting_fps()	#
			self.front_csi_right.start_counting_fps()
			
			self.recieve_slave_cameras()
			while not rospy.is_shutdown():
				#cams = CSI_Camera()
				#self.f1, self.f2, self.f3, self.f4 = self.recieve_slave_cameras()
				self.front_left_image=self.read_csi_camera(self.front_csi_left,self.show_fps, 'Front Left Camera')
				self.front_right_image=self.read_csi_camera(self.front_csi_right,self.show_fps, 'Front Right Camera')

				# Adding Grid Lines for Distance
				self.front_left_image, _, _, _ = self.draw_lines_symmetry_point(self.front_left_image, self.grid_l, (self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT), 'l')
				self.front_right_image, _, _, _ = self.draw_lines_symmetry_point(self.front_right_image, self.grid_r, (self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT), 'r')

				self.front_cam_stack = np.hstack((self.front_left_image, self.front_right_image))
				#self.front_cam_stack = np.hstack((self.front_right_image, self.front_left_image))
				if self.camera_view == 'front_view':
					self.f1 = cv2.resize(self.side_left_frame, (320, 240), interpolation=cv2.INTER_CUBIC)	# Change-1 Comment if you want to recieve four cameras as four streams
					self.f2 = cv2.resize(self.rear_left_frame, (320, 240), interpolation=cv2.INTER_CUBIC)	# Change-1 Comment if you want to recieve four cameras as four streams
					#print(self.front_cam_stack.shape)
					self.f3 = cv2.resize(self.side_right_frame, (320, 240), interpolation=cv2.INTER_CUBIC)	# Change-1 Comment if you want to recieve four cameras as four streams
					self.f4 = cv2.resize(self.rear_right_frame, (320, 240), interpolation=cv2.INTER_CUBIC)	# Change-1 Comment if you want to recieve four cameras as four streams
					if(len(self.f1.shape) == len(self.front_cam_stack.shape) and len(self.f2.shape) == len(self.front_cam_stack.shape) and len(self.f3.shape) == len(self.front_cam_stack.shape) and len(self.f4.shape) == len(self.front_cam_stack.shape)):			
						self.rear_side_left_cam = np.hstack((self.f1, self.f2))		# Change-1 Un-Comment if you want to recie	ve four cameras as four streams
						self.rear_side_right_cam = np.hstack((self.f4, self.f3))		# Change-1 Un-Comment if you want to recie	ve four cameras as four streams
						#print(self.side_rear_cams.shape)
						self.support_cams = np.hstack((self.rear_side_left_cam, self.rear_side_right_cam))
						self.six_cams_front_view = np.vstack((self.front_cam_stack, self.support_cams))		# Change-1 Comment if you want to recieve four cameras as four streams
						self.six_cams_pub = self.six_cams_front_view
						#self.pub.publish(self.br.cv2_to_imgmsg(self.six_cams_front_view, 'bgr8'))
 
					else:
						print("[INFO] Waiting for recieving the camera stream from Jetson NX front view")
						side_no_cam = np.zeros((240, 640,3), dtype="uint8")
						self.draw_error_status(side_no_cam, "No Image Availble", (100,100))
						self.draw_error_status(side_no_cam, "No Image Availble", (400,100))
						rear_no_cam = np.zeros((240, 640, 3), dtype="uint8")
						self.draw_error_status(rear_no_cam, "No Image Availble", (100,100))
						self.draw_error_status(rear_no_cam, "No Image Availble", (400,100))
						side_rear_cams_noimg = np.hstack((side_no_cam, rear_no_cam))
						#print(side_rear_cams_noimg.shape)
						six_cams_front_view_noimg = np.vstack((self.front_cam_stack, side_rear_cams_noimg))
						#print(six_cams_front_view_noimg.shape)
						self.six_cams_pub = six_cams_front_view_noimg
                                                #self.pub.publish(self.br.cv2_to_imgmsg(six_cams_front_view_noimg, 'bgr8'))
						#print("[INFO] Waiting for recieving the camera stream from Jetson NX ")
				elif self.camera_view == 'rear_view':
					self.f1 = cv2.resize(self.side_left_frame, (320, 240), interpolation=cv2.INTER_CUBIC)	# Change-1 Comment if you want to recieve four cameras as four streams
					self.f2 = cv2.resize(self.side_right_frame, (320, 240), interpolation=cv2.INTER_CUBIC)	# Change-1 Comment if you want to recieve four cameras as four streams
					self.f3 = cv2.resize(self.rear_left_frame, (640, 480), interpolation=cv2.INTER_CUBIC)	# Change-1 Comment if you want to recieve four cameras as four streams
					self.f4 = cv2.resize(self.rear_right_frame, (640, 480), interpolation=cv2.INTER_CUBIC)	# Change-1 Comment if you want to recieve four cameras as four streams
					self.f5 = cv2.resize(self.front_left_image, (320,240), interpolation=cv2.INTER_CUBIC)
					self.f6 = cv2.resize(self.front_right_image, (320,240), interpolation=cv2.INTER_CUBIC)


					if(len(self.f1.shape) == len(self.front_cam_stack.shape) and len(self.f2.shape) == len(self.front_cam_stack.shape) and len(self.f3.shape) == len(self.front_cam_stack.shape) and len(self.f4.shape) == len(self.front_cam_stack.shape)):
						self.front_side_cam_left = np.hstack((self.f2, self.f5))		# Change-1 Un-Comment if you want to recieve four cameras as four streams
						self.front_side_cam_right = np.hstack((self.f6, self.f1))		# Change-1 Un-Comment if you want to recieve four cameras as four streams
						self.front_side_cams = np.hstack((self.front_side_cam_left, self.front_side_cam_right))		# Change-1 Un-Comment if you want to recieve four cameras as four streams
						self.rear_cams_in_front = np.hstack((self.f3, self.f4))		# Change-1 Un-Comment if you want to recieve four cameras as four streams
						#print(self.front_side_cams.shape)
						self.six_cams_rear_view = np.vstack((self.rear_cams_in_front, self.front_side_cams))	
						#print(self.six_cams_rear_view.shape)
						self.six_cams_pub = self.six_cams_rear_view
                                                #elf.pub.publish(self.br.cv2_to_imgmsg(self.six_cams_rear_view, 'bgr8'))
						if (cv2.waitKey(20) & 0xFF) == 27:
							break 
					else:
						print("[INFO] Waiting for recieving the camera stream from Jetson NX rear view")
						side_no_cam = np.zeros((240, 640, 3), dtype="uint8")
						self.draw_error_status(side_no_cam, "No Image Availble", (100,100))
						self.draw_error_status(side_no_cam, "No Image Availble", (400,100))
						rear_no_cam = np.zeros((480, 1280, 3), dtype="uint8")
						self.draw_error_status(rear_no_cam, "No Image Availble", (200,200))
						self.draw_error_status(rear_no_cam, "No Image Availble", (900,200))
						f3_no_img = cv2.resize(self.front_cam_stack, (640,240), interpolation=cv2.INTER_CUBIC)
						side_rear_cams_noimg = np.hstack((side_no_cam, f3_no_img))
						six_cams_rear_view_noimg = np.vstack((rear_no_cam, side_rear_cams_noimg))
						self.six_cams_pub = six_cams_rear_view_noimg
                                                #self.pub.publish(self.br.cv2_to_imgmsg(six_cams_rear_view_noimg, 'bgr8'))
						self.pub.publish(self.br.cv2_to_imgmsg(self.six_cams_pub, 'bgr8'))
						self.virtualDeviceWriter.run(self.six_cams_pub)
				self.rate.sleep()
				#rospy.spin()

		finally:
			self.front_csi_left.stop()
			self.front_csi_left.release()
			self.front_csi_right.stop()
			self.front_csi_right.release()
		cv2.destroyAllWindows()


if __name__ == '__main__':

	cams = Stream_Cameras()
	cams.start_local_cameras()
	
