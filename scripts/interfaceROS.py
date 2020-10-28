#!/usr/bin/env python
# -*-coding:Latin-1 -*
from __future__ import print_function
import rospy


import param as p
import move as m

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist



class Robot_properties:

    def __init__(self):
        self.Angle_int = 0
        self.Angle_fi = 0
        self.Dist_rect = 0
        self.STOP = False

        self.publish_Distance = '/odriveDistance_parcourue'
        self.publish_Vitesse0 = '/odriveVitesse_RG'  # nom du topic que je publie
        self.publish_Vitesse1 = '/odriveVitesse_RD'
        self.publish_Position_atteinte = '/odrivePosition_atteinte'

        self.pubDistance = rospy.Publisher(self.publish_Distance, Float32, queue_size=1)
        self.pubVitesse0 = rospy.Publisher(self.publish_Vitesse0, Twist, queue_size=1)
        self.pubVitesse1 = rospy.Publisher(self.publish_Vitesse1, Twist, queue_size=1)
        self.pubPosition_atteinte = rospy.Publisher(self.publish_Position_atteinte, Bool, queue_size=1)

        rospy.Subscriber("Angle_intermediaire", Float32, self.update_Angle_int)
        rospy.Subscriber("Angle_final", Float32, self.update_Angle_fi)
        rospy.Subscriber("Distance_rectiligne", Float32, self.update_Distance_rec)
        rospy.Subscriber("Stop_time", Bool, self.update_Stop)

    """ INPUTS """

    def update_Angle_int(self, data):
        self.Angle_int = data.data

    def update_Angle_fi(self, data):
        self.Angle_fi = data.data

    def update_Distance_rec(self, data):
        self.Dist_rect = data.data

    def update_Stop(self, data):
        self.STOP = data.data

    """ OUTPUTS """

    def update_Vitesse0(self, vitesse0):
        # convertir en Twist
        toTwist = Twist()
        toTwist.linear.x = vitesse0
        # print("vitesse roue gauche en m/s: " % vitesse0)
        self.pubVitesse0.publish(toTwist)

    def update_Vitesse1(self, vitesse1):
        # convertir en Twist
        toTwist = Twist()
        toTwist.linear.x = vitesse1
        # print("vitesse roue droite en m/s: " % vitesse1)
        self.pubVitesse1.publish(toTwist)

    def update_Distance_parc(self, Distance):
        # convertir en float32
        toFloat32 = Float32()
        toFloat32.data = Distance
        # print("Distance parcourue en mm :" % Distance)
        self.pubDistance.publish(toFloat32)

    def update_Position_atteinte(self, pos_atteinte):

        toBool = Bool()
        toBool.data = pos_atteinte
        self.pubPosition_atteinte.publish(toBool)


def main():
    rospy.init_node('Odrive', anonymous=True)
    param = p.Param()
    param.config()
    #param.calib()
    #param.raz_encoders()
    move = m.Move(param.odrv)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        move.run()
        rospy.sleep(1)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
