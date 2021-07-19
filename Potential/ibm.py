#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
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

    def datacallback(self, msg):
        self.yaw = msg.yaw
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.longitude = msg.longitude
        self.latitude = msg.latitude
        self.angle = msg.angle


    def transfer(self):
        msg = Float64MultiArray()
        msg.data = [self.yaw, self.roll, self.pitch, self.longitude, self.latitude, self.angle]
        publish(msg)

if __name__ == '__main__':
    print("Start")
    rospy.init_node('IoT', anonymous=True)
    rate = rospy.Rate(3) # 1hz
    server = Server()

    rospy.Subscriber('realData', ShipData, server.datacallback)

    while not rospy.is_shutdown():
        server.transfer()
        rate.sleep()
    rospy.spin()