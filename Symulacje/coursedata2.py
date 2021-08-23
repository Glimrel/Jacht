#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from sandbox.msg import Courseerror
from sandbox.msg import Objdata

def publish(msg):
    publisher = rospy.Publisher('CourseFuzzyHub', Courseerror,queue_size=10)
    rospy.loginfo(msg)
    publisher.publish(msg)

def pubwind(msg):
    publisher = rospy.Publisher('SailFuzzyHub', Int32 ,queue_size=10)
    rospy.loginfo(msg)
    publisher.publish(msg)

class Server:
    def __init__(self):
        self.sp = 0
        self.e = 0
        self.de = 0
        self.pv = 0
        self.epop = 0
        self.windd = 0
        self.windv = 0

    def spcallback(self, msg):
        self.sp = msg.data

    def datacallback(self, msg):
        self.pv = msg.course_angle
        self.de = msg.course_change_rate
        self.windd = msg.wind_direction
        self.windv = msg.wind_velocity

    def calculate_error(self):
        msg = Courseerror()

        self.e = (float(self.sp) - self.pv) * 10
        #self.de = (self.e - self.epop) * 10
        self.epop = self.e

        if self.e > 50:
            msg.error = 50
        elif self.e < -50:
            msg.error = -50
        else:
            msg.error = self.e

        if self.de > 50:
            msg.errorDerivative = 50
        elif self.de < -50:
            msg.errorDerivative = -50
        else:
            msg.errorDerivative = self.de

        publish(msg)

        x = self.windd - int(self.pv)

        if x < 0:
            x = 360+x

        if x <= 180:
            x = x

        if x > 180:
            x = 360-x

        if x < 0:
            x = -x

        pubwind(x)


if __name__ == '__main__':
    print("Start")
    rospy.init_node('Capitan', anonymous=True)
    rate = rospy.Rate(0.5) # 1hz
    server = Server()

    rospy.Subscriber('symulationData', Objdata, server.datacallback)
    rospy.Subscriber('SP', Int32, server.spcallback)

    while not rospy.is_shutdown():
        server.calculate_error()
        rate.sleep()

    rospy.spin()