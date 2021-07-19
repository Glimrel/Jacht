#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float64MultiArray, Int32
import datetime


class Server:
    def __init__(self):
        self.yaw = 0
        self.roll = 0
        self.pitch = 0
        self.longitude = 0
        self.latitude = 0
        self.angle = 0
        self.longitudeSP = 0
        self.latitudeSP = 0
        self.sail = 0
        self.rudder = 0
        self.setPoint = 0

    def liveData_callback(self, msg):
        self.yaw = msg.data[0]
        self.roll = msg.data[1]
        self.pitch = msg.data[2]
        self.longitude = msg.data[3]
        self.latitude = msg.data[4]
        self.angle = msg.data[5]

    def longSP_callback(self, msg):
        self.longitudeSP = msg.data

    def latiSP_callback(self, msg):
        self.latitudeSP = msg.data

    def sail_callback(self, msg):
        self.sail = msg.data

    def rudder_callback(self, msg):
        self.rudder = msg.data

    def setPoint_callback(self, msg):
        self.setPoint = msg.data

    def updateRecord(self,file):
        time = datetime.datetime.now()
        file.write("%s/%s/%s " % (time.day, time.month, time.year))
        file.write("%s:%s:%s " % (time.hour, time.minute, time.second))
        file.write("%s %s %s " % (self.yaw, self.roll, self.pitch))
        file.write("%s %s %s " % (self.longitude, self.latitude, self.angle))
        file.write("%s %s " % (self.longitudeSP, self.latitudeSP))
        file.write("%s %s %s\n" % (self.sail, self.rudder, self.setPoint))

if __name__ == '__main__':
    print("Start")
    rospy.init_node('Record', anonymous=True)
    rate = rospy.Rate(3) #hz
    server = Server()

    rospy.Subscriber('LongitudeFromRED', Float64, server.longSP_callback)
    rospy.Subscriber('LatitudeFromRED', Float64, server.latiSP_callback)
    rospy.Subscriber('toIBM', Float64MultiArray, server.liveData_callback)
    rospy.Subscriber('SetSail', Int32 , server.sail_callback)
    rospy.Subscriber('SetRudder', Int32, server.rudder_callback)
    rospy.Subscriber('setPoint', Float64, server.setPoint_callback)

    file = open("recordedData.txt","a")
    time = datetime.datetime.now()
    print("Start recording at %s" % time)
    file.write("Start recording at %s \n" % time)
    file.write("data time yaw roll pitch longitude latitude angle longitudeSP latitudeSP sail rudder setPoint \n")

    while not rospy.is_shutdown():
        server.updateRecord(file)
        rate.sleep()
    rospy.spin()