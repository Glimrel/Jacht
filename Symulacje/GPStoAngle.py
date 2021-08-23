#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sandbox.msg import ShipData

from math import cos
from math import sin
from math import atan2
from math import degrees

def pubcourse(msg):
    publisher = rospy.Publisher('setPoint', Float64, queue_size=10)
    rospy.loginfo(msg)
    publisher.publish(msg)

class Server:
    def __init__(self):
        self.DA = 0
        self.SA = 0
        #self.DA = 54.244391
        #self.SA = 18.845647
        self.DB = 0
        self.SB = 0
        #self.DB = 54.244391
        #self.SB = 18.845647

    def GPScallback(self, msg):
        self.DA = msg.longitude/100
        self.SA = msg.latitude/100

    def LongSPcallback(self, msg):
        self.DB = msg.data


    def LatiSPcallback(self, msg):
        self.SB = msg.data


    def calculateCourse(self):
        # D - dlugosc geograficzna (longitude)
        # S - szerokosc geograficzna (latitude)
        # A - punkt A (nasza pozycja)	
        # B - punkt B (pozycja docelowa)
        # K - kierunek
        # d - delta
        print(self.DA, self.SA)
        print(self.DB, self.SB)
        dD = self.DA - self.DB
        X = cos(self.SB)*sin(dD)
        Y = cos(self.SA)*sin(self.SB) - sin(self.SA)*cos(self.SB)*cos(dD)
        K = atan2(X, Y)
        K = degrees(K)
        pubcourse(K)


if __name__ == '__main__':
    print("Start")
    rospy.init_node('Setter', anonymous=True)
    rate = rospy.Rate(3) # 1hz
    server = Server()

    rospy.Subscriber('realData', ShipData, server.GPScallback)
    rospy.Subscriber('LongitudeFromRED', Float64, server.LongSPcallback)
    rospy.Subscriber('LatitudeFromRED', Float64, server.LatiSPcallback)

    while not rospy.is_shutdown():
        server.calculateCourse()
        rate.sleep()

    rospy.spin()