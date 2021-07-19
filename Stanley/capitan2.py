#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from sandbox.msg import ShipData

def pubwind(msg):
    publisher = rospy.Publisher('SailFuzzyHub', Int32 ,queue_size=10)
    rospy.loginfo(msg)
    publisher.publish(msg)

class Server:
    def __init__(self):
        self.wind = 0

    def datacallback(self, msg):
        self.wind = msg.angle

    def calculate_wind(self):
        wind = self.wind
        if wind > 180:
            wind = 360-wind
        windAngleMsg = abs(wind)

        pubwind(windAngleMsg)


if __name__ == '__main__':
    print("Start")
    rospy.init_node('Capitan', anonymous=True)
    rate = rospy.Rate(3) # 1hz
    server = Server()

    rospy.Subscriber('realData', ShipData, server.datacallback)

    while not rospy.is_shutdown():
        server.calculate_wind()
        rate.sleep()

    rospy.spin()
