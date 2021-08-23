#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float64MultiArray, Int32
from sandbox.msg import ShipData


def publish(msg):
    publisher = rospy.Publisher('toNodeRED', Float64MultiArray ,queue_size=10)
    rospy.loginfo(msg)
    publisher.publish(msg)

class Server:
    def __init__(self):
        self.yaw = 0
        self.roll = 0
        self.pitch = 0
        self.longitude = 0
        self.latitude = 0
        self.angle = 0
        self.sail = 50
        self.rudder = 50
        self.setPoint = 1

    def datacallback(self, msg):
        self.yaw = msg.yaw
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.longitude = msg.longitude
        self.latitude = msg.latitude
        self.angle = msg.angle
        
    def sail_callback(self, msg):
        self.sail = msg.data

    def rudder_callback(self, msg):
        self.rudder = msg.data

    def setPoint_callback(self, msg):
        self.setPoint = msg.data

    def transfer(self):
        msg = Float64MultiArray()
        msg.data = [self.yaw, self.roll, self.pitch, self.longitude, self.latitude, self.angle, self.setPoint, self.rudder, self.sail]
        publish(msg)

if __name__ == '__main__':
    print("Start")
    rospy.init_node('IoT', anonymous=True)
    rate = rospy.Rate(3) # 1hz
    server = Server()

    rospy.Subscriber('realData', ShipData, server.datacallback)
    rospy.Subscriber('SetSail', Int32 , server.sail_callback)
    rospy.Subscriber('SetRudder', Int32, server.rudder_callback)
    rospy.Subscriber('setPoint', Float64, server.setPoint_callback)

    while not rospy.is_shutdown():
        server.transfer()
        rate.sleep()
    rospy.spin()