#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html
from imutils.video import VideoStream
import face_recognition
import argparse
import imutils
import pickle
import time
import cv2
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-e", "--encodings", required=True,
	help="path to serialized db of facial encodings")


ap.add_argument("-d", "--detection-method", type=str, default="cnn",
	help="face detection model to use: either `hog` or `cnn`")
args = vars(ap.parse_args())

# Instantiate CvBridge


# load the known faces and embeddings
print("[INFO] loading encodings...")
data = pickle.loads(open(args["encodings"], "rb").read())

# initialize the video stream and pointer to output video file, then
# allow the camera sensor to warm up
print("[INFO] starting video stream...")

bridge = CvBridge()
count = 1
patient_names = []
def image_callback(msg):
    
    global count
    #print("Image Callback")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        # checked --- no delay until here
    except CvBridgeError, e:
        print(e)
    else:
        if (count%24 == 0 or count == 1):
		# Save your OpenCV2 image as a jpeg 
		time = msg.header.stamp
		#cv2.imwrite('/home/sanjuna/mybot_ws/src/mob_rob_new_9_moveit/scripts/deep learning/face-recognition-opencv/examples/example_04.png', cv2_img)
		rgb = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
		rgb = imutils.resize(cv2_img, width=750)
		r = cv2_img.shape[1] / float(rgb.shape[1])
		# detect the (x, y)-coordinates of the bounding boxes
		# corresponding to each face in the input cv2_img, then compute
		# the facial embeddings for each face
		boxes = face_recognition.face_locations(rgb,
			model=args["detection_method"])
		encodings = face_recognition.face_encodings(rgb, boxes)
		names = []

		# loop over the facial embeddings
		for encoding in encodings:
			# attempt to match each face in the input image to our known
			# encodings
			matches = face_recognition.compare_faces(data["encodings"],
				encoding)
			name = "Unknown"

			# check to see if we have found a match
			if True in matches:
				# find the indexes of all matched faces then initialize a
				# dictionary to count the total number of times each face
				# was matched
				matchedIdxs = [i for (i, b) in enumerate(matches) if b]
				counts = {}

				# loop over the matched indexes and maintain a count for
				# each recognized face face
				for i in matchedIdxs:
					name = data["names"][i]
					counts[name] = counts.get(name, 0) + 1

				# determine the recognized face with the largest number
				# of votes (note: in the event of an unlikely tie Python
				# will select first entry in the dictionary)
				name = max(counts, key=counts.get)
		
			# update the list of names
			names.append(name)
                        print(patient_names)
			if(name not in patient_names):#name_count == 0 and name != "Unknown"
				soundhandle = SoundClient()
		    		rospy.sleep(2)
		    		voice = 'voice_kal_diphone'
		    		volume = 1.0
		    		s = "Hello "+name
		    		print 'Greeting: %s' % s
		    		print 'Voice : %s' % voice
		   		print 'Volume level: %s' % volume

				soundhandle.say(s, voice, volume)
                        	rospy.sleep(1)
                                
                                patient_names.append(name)

                                with open('/home/sanjuna/mybot_ws/src/mob_rob_new_9_moveit/patients_inspected/patient_names.txt', 'w') as file:
				   
				   file.write(str(patient_names))
				   rospy.loginfo("Written patient name to patient_names.txt file")    
			##########need to publish name as string.. which will be the argument to sound_play package######################

		# loop over the recognized faces
		for ((top, right, bottom, left), name) in zip(boxes, names):
			# rescale the face coordinates
			top = int(top * r)
			right = int(right * r)
			bottom = int(bottom * r)
			left = int(left * r)

			# draw the predicted face name on the image
			cv2.rectangle(cv2_img, (left, top), (right, bottom),
				(0, 255, 0), 2)
			y = top - 15 if top - 15 > 15 else top + 15
			cv2.putText(cv2_img, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
				0.75, (0, 255, 0), 2)

		# check to see if we are supposed to display the output frame to
		# the screen
		cv2.imshow('',cv2_img)
		cv2.waitKey(1)
	
	count = count + 1
        #print (count)
	
def main():
    rospy.init_node('image_listener')
    # Define your image topic
    #image_topic = "/camera/rgb/image_raw"
    image_topic = "/usb_cam/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback, queue_size = 1, buff_size=2**24)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
