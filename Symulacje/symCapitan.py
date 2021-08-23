#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Int32
from sandbox.msg import Courseerror
from sandbox.msg import Objdata

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
        self.epop = 0
        self.pv = 0
        self.windd = 0
        self.windv = 0

    def spcallback(self, msg):
        self.sp = msg.data

    def datacallback(self, msg):
        self.pv = msg.course_angle
        self.windd = msg.wind_direction
        self.windv = msg.wind_velocity
        server.calculate_error()

    def calculate_error(self):
        courseErrorMsg = Courseerror()
        setponint = self.sp
        procesvalue = self.pv
       
        #Przypadek przekroczenia 180 (nawrot)
        if abs(setponint - procesvalue) > 180:
            print('Przekroczono 180')
            if setponint > 0:
                setponint = setponint - 180
            else:
                setponint = setponint + 180
            if procesvalue > 0:
                procesvalue = procesvalue - 180
            else:
                procesvalue = procesvalue + 180
        
        self.e = (setponint - procesvalue) * 5
        self.de = (self.e - self.epop) * 4
        self.epop = self.e
        
        print('blad', self.e, 'pochodna bledu', self.de)
        
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
            
        pubcourse(courseErrorMsg)

        x = self.windd - int(self.pv)

        if x < 0:
            x = 360+x

        if x <= 180:
            x = x

        if x > 180:
            x = 360-x

        if x < 0:
            x = -x
        
        print('Wiatr',x)
        pubwind(x)
    
if __name__ == '__main__':
    print("Start")
    rospy.init_node('Capitan', anonymous=True)
    rate = rospy.Rate(0.5) # 1hz
    server = Server()

    rospy.Subscriber('symulationData', Objdata, server.datacallback)
    rospy.Subscriber('setPoint', Float64, server.spcallback)

    while not rospy.is_shutdown():
        
        rate.sleep()

    rospy.spin()
