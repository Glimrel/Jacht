#!/usr/bin/env python

import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from sandbox.msg import Objdata

    
def pubcourse(msg):
    publisher = rospy.Publisher('setPoint', Float64, queue_size=10)
    publisher.publish(msg)  

class Point:
  def __init__(self, x, y):
        self.x = x
        self.y = y

#Odleglosc Euklidesowa
def dist(P, Q):
  dist = math.sqrt((Q.x-P.x)**2 + (Q.y-P.y)**2)
  return dist

class Goal:
  Gg = 3
  def Potential(self, position, goal):
    Pg = self.Gg * dist(position, goal)
    return Pg

class Obstacle:
  k = 100
  def Potential(self, position, obstacle):
    if dist(position, obstacle) != 0:
      Po = self.k/dist(position, obstacle)
    else:
      Po = 200
    return Po

class Upwind:
  Gup = 10
  def Potential(self, boat, position, windDirection):
    angle = math.degrees(math.atan2(position.y - boat.y, position.x - boat.x))
    fi = windDirection - angle

    if fi < -180:
      fi = 360 + fi
    if fi > 180:
      fi = 360 - fi

    if 0 <= abs(fi)< 60:
      Pup = self.Gup*dist(boat, position)
    else:
      Pup = 0
    return Pup

class Downwind:
  Gdown = 10
  def Potential(self, boat, position, windDirection):
    angle = math.degrees(math.atan2(position.y - boat.y, position.x - boat.x))
    fi = windDirection - angle

    if fi < -180:
      fi = 360 + fi
    if fi > 180:
      fi = 360 - fi

    if 0<= abs(abs(fi)-180) < 30:
      Pdown = self.Gdown * dist(boat, position)
    else:
      Pdown = 0
    return Pdown

class AngleChange:
  Gh = 0.03
  def Potential(self, boat, position, windDirection, previousDirection):
    angle = math.degrees(math.atan2(position.y - boat.y, position.x - boat.x))
    Ph = self.Gh * abs(previousDirection - angle)
    return Ph

class Server:
    def __init__(self):
        self.longitudePV = 100
        self.latitudePV = 100
        self.longitudeSP = 200
        self.latitudeSP = 200
        self.previousDirection = 0
        self.windDirection = 0
        
    def Data_callback(self, msg):
        self.longitudePV = msg.x
        self.latitudePV = msg.y
        self.previousDirection = msg.course_angle
        self.windDirection = msg.wind_direction

    def LongSP_callback(self, msg):
        self.longitudeSP = msg.data
        #print('Zmiana SP longitude na: ', self.longitudeSP)

    def LatiSP_callback(self, msg):
        self.latitudeSP = msg.data
        #print('Zmiana SP latitude na: ', self.latitudeSP)
        
    def loop(self):
        windD = self.windDirection
        course = self.previousDirection
        boat = Point(self.longitudePV,self.latitudePV)
        missionGoal = Point(self.longitudeSP, self.latitudeSP)
        obstacle = Point(0,0) #Deklaracja przeszkody poza obszarem
        yaw_ref = windD
        
        window = np.zeros(180)
        X = np.zeros(180)       
        Y = np.zeros(180)
        
        GoalObject = Goal()
        ObstacleObject = Obstacle()  
        UpwindObject = Upwind() 
        DownwindObject = Downwind() 
        AngleChangeObject = AngleChange()
        
        if dist(boat, missionGoal) > 2: 
            radius = 1  
            index = 0
            for angle in range(0, 360, 2):
              x = boat.x + radius*math.cos(math.radians(angle))
              y = boat.y + radius*math.sin(math.radians(angle))
              position = Point(x, y)
              GoalP = GoalObject.Potential(position, missionGoal)
              ObstacleP = ObstacleObject.Potential(position, obstacle)
              UpwindP = UpwindObject.Potential(boat, position, windD)
              DownwindP = DownwindObject.Potential(boat, position, windD)
              AngleChangeP = AngleChangeObject.Potential(boat, position, windD, course)
              Potential = GoalP + ObstacleP + UpwindP + DownwindP + AngleChangeP
              window[index] = Potential
              X[index] = x
              Y[index] = y
              index = index + 1

            minIndex = np.argmin(window) #indeks dla minimum na okregu
            #X[minIndex] pozadana zmiana pozycji statku na wyznaczone minimum
            #Y[minIndex] pozadana zmiana pozycji statku na wyznaczone minimum
            yaw_ref = math.degrees(math.atan2(Y[minIndex]  - boat.y, X[minIndex]- boat.x))    
        print('Obliczona wartosc zadana kursu wynosi:', yaw_ref)
        print('Aktualny cel: ',self.longitudeSP, self.latitudeSP)
        pubcourse(yaw_ref)

if __name__ == '__main__':
    print('start')
    rospy.init_node('PotientialField', anonymous=True)
    rate = rospy.Rate(0.2) # 1hz
    server = Server()
    
    rospy.Subscriber('symulationData', Objdata, server.Data_callback)
    rospy.Subscriber('LongitudeFromRED', Float64, server.LongSP_callback)
    rospy.Subscriber('LatitudeFromRED', Float64, server.LatiSP_callback)
    
    while not rospy.is_shutdown():
        server.loop()
        rate.sleep()
        
    rospy.spin()


