#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from sandbox.msg import Courseerror
from sandbox.msg import ShipData

def pubcourse(msg):
    publisher = rospy.Publisher('CourseFuzzyHub', Courseerror,queue_size=10)
    #rospy.loginfo(msg)
    publisher.publish(msg)

def pubwind(msg):
    publisher = rospy.Publisher('SailFuzzyHub', Int32 ,queue_size=10)
    #rospy.loginfo(msg)
    publisher.publish(msg)

class Server:
    def __init__(self):
        self.sp = 0
        self.e = 0
        self.de = 0
        self.pv = 0
        self.epop = 0
        self.wind = 0

    def spcallback(self, msg):
        self.sp = msg.data

    def datacallback(self, msg):
        self.pv = msg.yaw
        self.wind = msg.angle

    def calculate_error(self):
        courseErrorMsg = Courseerror()
        setponint = self.sp
        procesvalue = self.pv

        #Przypadek przekroczenia 180 (nawrot)
        if abs(setponint - procesvalue) > 180:
            if setponint > 0:
                setponint = setponint - 180
            else:
                setponint = setponint + 180
            if procesvalue > 0:
                procesvalue = procesvalue - 180
            else:
                procesvalue = procesvalue + 180

        #Obliczenie uchybu i pochodnej
        self.e = (setponint - procesvalue) * 10 # waga uchybu - skalowanie fuzzy
        self.de = (self.e - self.epop) * 10 #waga pochodnej uchybu - skalowanie fuzzy
        self.epop = self.e


        #Ograniczenia na wartosci uchybu i pochodnej uchybu
        if self.e > 50:
            courseErrorMsg.error = 50
        elif self.e < -50:
            courseErrorMsg.error = -50
        else:
            courseErrorMsg.error = self.e

        if self.de > 50:
            courseErrorMsg.errorDerivative = 50
        elif self.de < -50:
            courseErrorMsg.errorDerivative = -50
        else:
            courseErrorMsg.errorDerivative = self.de

        wind = self.wind
        if wind > 180:
            wind = 360-wind
        windAngleMsg = abs(wind)

        pubcourse(courseErrorMsg)
        pubwind(windAngleMsg)


if __name__ == '__main__':
    print("Start")
    rospy.init_node('Capitan', anonymous=True)
    rate = rospy.Rate(3) # 1hz
    server = Server()

    rospy.Subscriber('realData', ShipData, server.datacallback)
    rospy.Subscriber('setPoint', Float64, server.spcallback)

    while not rospy.is_shutdown():
        server.calculate_error()
        rate.sleep()

    rospy.spin()