#!/usr/bin/env python


import sys
import os
import rospy

import comArduino

from std_msgs.msg import String as ROS_String
from geometry_msgs.msg import Pose as ROS_Pose
from std_msgs.msg import String
from std_msgs.msg import Int16

class Robot_properties:
	def __init__(self):
		self.messageArduino = ""

		self.publish_arduino = 'arduinoOrder' #nom du topic que je publie
		#self.pubArduino = rospy.Publisher(self.publish_arduino, String, queue_size=10)
		self.pubArduino = rospy.Publisher(self.publish_arduino, Int16, queue_size=10)
		
		rospy.Subscriber("/arduinoState", Int16, self.subscrib)	

	def subscrib(self, data):
		self.messageArduino = data

	def publish(self, message): 
		print message
		self.pubArduino.publish(message)

def main():
	rospy.init_node('Traite_arduinoProg', anonymous=True)
	Objtester = comArduino.TesterArduino() #objet de type objet
	while not rospy.is_shutdown():
		Objtester.updater()
		rospy.sleep(0.03)

if __name__ ==  '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
