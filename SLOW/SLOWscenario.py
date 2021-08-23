#!/usr/bin/env python

import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from sandbox.msg import ShipData

def getDistanceInMeters(pointA, pointB):
  earthRadius = 6371000
  latA = pointA.y * math.pi/180
  latB = pointB.y * math.pi/180
  diffLat = (pointA.y - pointB.y) * math.pi/180
  diffLng = (pointA.x - pointB.x) * math.pi/180
  a = math.sin(diffLat / 2) * math.sin(diffLat / 2) + math.cos(latB) * math.cos(latA) * math.sin(diffLng / 2) * math.sin(diffLng / 2)
  b = 2 * math.atan2( math.sqrt(a), math.sqrt(1-a) )
  distance = round((earthRadius * b), 2)
  return distance

def pubLong(msg):
    publisher = rospy.Publisher('LongitudeFromRED', Float64, queue_size=10)
    publisher.publish(msg)
    
def pubLati(msg):
    publisher = rospy.Publisher('LatitudeFromRED', Float64, queue_size=10)
    publisher.publish(msg)
    
class Point:
  def __init__(self, x, y):
        self.x = x
        self.y = y

class Server:
    def __init__(self):
        self.longitudePV = 17.950000
        self.latitudePV = 54.210000
        self.longitudeSP = 17.954909
        self.latitudeSP = 54.213426
        self.counter = 3
        
    def Data_callback(self, msg):
        self.longitudePV = msg.longitude/100
        self.latitudePV = msg.latitude/100
        
    def loop(self):      
        goal = Point(self.longitudeSP , self.latitudeSP)
        position = Point(self.longitudePV , self.latitudePV)
        dist = getDistanceInMeters(goal, position)
        print('Odleglosc od celu = ', dist)
        print('Aktualny cel (Long, Lati)', self.longitudeSP , self.latitudeSP)
        
        if dist < 5 and self.counter == 0:
            self.longitudeSP = 17.955029
            self.latitudeSP = 54.211931
            self.counter = 1
            print("Osignito cel, wyznaczono nowy cel Long, Lati", self.longitudeSP , self.latitudeSP)
            goal = Point(self.longitudeSP , self.latitudeSP)
            dist = getDistanceInMeters(goal, position)
        
        if dist < 5 and self.counter == 1:
            self.longitudeSP = 17.956900
            self.latitudeSP = 54.212635
            self.counter = 2
            print("Osignito cel, wyznaczono cel Long, Lati", self.longitudeSP , self.latitudeSP)
            goal = Point(self.longitudeSP , self.latitudeSP)
            dist = getDistanceInMeters(goal, position)
            
        if dist < 5 and self.counter == 2:
            self.longitudeSP = 17.954909
            self.latitudeSP = 54.213426
            self.counter = 0
            print("Osignito cel, wyznaczono cel Long, Lati", self.longitudeSP , self.latitudeSP)
            goal = Point(self.longitudeSP , self.latitudeSP)
            dist = getDistanceInMeters(goal, position)
            
        pubLong(self.longitudeSP)
        pubLati(self.latitudeSP)

if __name__ == '__main__':
    print('start')
    rospy.init_node('Scenario', anonymous=True)
    rate = rospy.Rate(0.2) # 1hz
    server = Server()
    
    rospy.Subscriber('realData', ShipData, server.Data_callback)
    
    while not rospy.is_shutdown():
        server.loop()
        rate.sleep()
        
    rospy.spin()
