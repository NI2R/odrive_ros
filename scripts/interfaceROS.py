#!/usr/bin/env python


import sys
import os
import rospy
from math import pi

import param  # a verifier


from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class Robot_properties:

	def __init__(self):
		self.Angle_int = 0
		self.Angle_fi = 0
		self.Dist_rect = 0

		self.publish_Distance = 'odriveDistance_parcourue'
		self.publish_Vitesse0 = 'odriveVitesse0' #nom du topic que je publie
		self.publish_Vitesse1 = 'odriveVitesse1'

		self.pubDistance = rospy.Publisher(self.publish_Distance, Float32, queue_size=1)
		self.pubVitesse0 = rospy.Publisher(self.publish_Vitesse0, Twist, queue_size=1)
		self.pubVitesse1 = rospy.Publisher(self.publish_Vitesse1, Twist, queue_size=1)


		rospy.Subscriber("Angle_intermediaire", Float32, self.update_Angle_int)
		rospy.Subscriber("Angle_final", Float32, self.update_Angle_fi)
		rospy.Subscriber("Distance_rectiligne", Float32, self.update_Distance_rec)

	def update_Angle_int(self, data):
		#self. = data
		self.Angle_int = data * pi / 180

	def update_Angle_fi(self, data):
		#self. = data
		self.Angle_fi = data * pi / 180

	def update_Distance_rec(self, data):
		#self. = data
		self.Dist_rect = data

	def update_Vitesse0(self, vitesse0):
		print vitesse0
		# convertir en Twist
		self.pubVitesse0.publish(vitesse0)

	def update_Vitesse1(self, vitesse1):
		print vitesse1
		# convertir en Twist
		self.pubVitesse1.publish(vitesse1)

	def update_Distance_parc(self, Distance):
		print Distance
		# convertir en Twist
		self.pubDistance.publish(Distance)



def main():
	param = param.Param()
	rospy.init_node('Odrive', anonymous=True)
	param.config()
	param.calib()
	while not rospy.is_shutdown():
		#Objtester.updater()
		rospy.sleep(1)

if __name__ ==  '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
