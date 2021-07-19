#!/usr/bin/env python

import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from sandbox.msg import Objdata

def pubLong(msg):
    publisher = rospy.Publisher('LongitudeFromRED', Float64, queue_size=10)
    #rospy.loginfo(msg)
    publisher.publish(msg)
    
def pubLati(msg):
    publisher = rospy.Publisher('LatitudeFromRED', Float64, queue_size=10)
    #rospy.loginfo(msg)
    publisher.publish(msg)
    
class Point:
  def __init__(self, x, y):
        self.x = x
        self.y = y

#Odleglosc Euklidesowa
def dist(P, Q):
  dist = math.sqrt((Q.x-P.x)**2 + (Q.y-P.y)**2)
  return dist

class Server:
    def __init__(self):
        self.longitudePV = 100
        self.latitudePV = 100
        self.longitudeSP = 400
        self.latitudeSP = 400
        self.counter = 0
        
    def Data_callback(self, msg):
        self.longitudePV = msg.x
        self.latitudePV = msg.y
        
    def loop(self):
        
        goal = Point(self.longitudeSP , self.latitudeSP)
        position = Point(self.longitudePV , self.latitudePV)
        
        if dist(goal, position) < 20 and self.counter == 0:
            self.longitudeSP = 500.0
            self.latitudeSP = 300.0
            self.counter = 1
            print("Osignito cel, wyznaczono nowy cel", self.longitudeSP , self.latitudeSP)
            goal = Point(self.longitudeSP , self.latitudeSP)
        
        if dist(goal, position) < 20 and self.counter == 1:
            self.longitudeSP = 300.0
            self.latitudeSP = 100.0
            self.counter = 2
            print("Osignito cel, wyznaczono nowy cel", self.longitudeSP , self.latitudeSP)
            goal = Point(self.longitudeSP , self.latitudeSP)
            
        if dist(goal, position) < 20 and self.counter == 2:
            self.longitudeSP = 100.0
            self.latitudeSP = 100.0
            self.counter = 3
            print("Osignito cel, wyznaczono nowy cel", self.longitudeSP , self.latitudeSP)
            goal = Point(self.longitudeSP , self.latitudeSP)
            
        if dist(goal, position) < 20 and self.counter == 3:
            self.longitudeSP = 100.0
            self.latitudeSP = 200.0
            self.counter = 0
            print("Osignito cel, wyznaczono nowy cel", self.longitudeSP , self.latitudeSP)
            goal = Point(self.longitudeSP , self.latitudeSP)
        pubLong(self.longitudeSP)
        pubLati(self.latitudeSP)
        

if __name__ == '__main__':
    print('start')
    rospy.init_node('WaypointsSetter', anonymous=True)
    rate = rospy.Rate(0.2) # 1hz
    server = Server()
    
    rospy.Subscriber('symulationData', Objdata, server.Data_callback)
    
    while not rospy.is_shutdown():
        server.loop()
        rate.sleep()
        
    rospy.spin()