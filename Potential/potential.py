#!/usr/bin/env python

import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from sandbox.msg import ShipData

    
def pubcourse(msg):
    publisher = rospy.Publisher('setPoint', Float64, queue_size=10)
    rospy.loginfo(msg)
    publisher.publish(msg)  

class Point:
  def __init__(self, x, y):
        self.x = x
        self.y = y

#Odleglosc Euklidesowa
def dist(P, Q):
  dist = math.sqrt((Q.x-P.x)**2 + (Q.y-P.y)**2)
  return dist

def getDistanceInMeters(pointA, pointB):
  earthRadius = 6371000
  latA = pointA.y * math.pi/180
  latB = pointB.y * math.pi/180
  diffLat = (pointA.y - pointB.y) * math.pi/180
  diffLng = (pointA.x - pointB.x) * math.pi/180
  a = math.sin(diffLat / 2) * math.sin(diffLat / 2) + math.cos(latB) * math.cos(latA) * math.sin(diffLng / 2) * math.sin(diffLng / 2)
  #b = 2 * math.asin(math.sqrt(a))
  b = 2 * math.atan2( math.sqrt(a), math.sqrt(1-a) )
  distance = round((earthRadius * b), 2)
  return distance

#Potencjaly
class Potentials:
  def __init__(self, Gg=3, k=100, Gup=10, Gdown=5, Gh=0.0000004):
    self.Gg = Gg
    self.k = k
    self.Gup = Gup
    self.Gdown = Gdown
    self.Gh = Gh

  #Potencjal wzgledem celu
  def PotentialGoal(self, position, goal):
      Pg = self.Gg * dist(position, goal)
      return Pg
  
  #Potencjal wzgledem przeszkody
  def PotentialObstacle(self, position, obstacle):
    if dist(position, obstacle) != 0:
      Po = self.k/dist(position, obstacle)
    else:
      Po = 200
    return Po

  #Potencjal pod wiatr
  def PotentialUpwind(self, boat, position, windDirection):
    angle = math.degrees(math.atan2(position.y - boat.y, position.x - boat.x))
    fi = windDirection - angle

    if fi < -180:
      fi = 360 + fi
    if fi > 180:
      fi = 360 - fi

    if 0 <= abs(fi)< 30:
      Pup = self.Gup*dist(boat, position)
    else:
      Pup = 0
    return Pup

  #Potencjal z wiatrem
  def PotentialDownwind(self, boat, position, windDirection):
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

  #Potencjal- kara za zmiane kierunku
  def PotentialAngleChange(self, boat, position, windDirection, previousDirection):
      angle = math.degrees(math.atan2(position.y - boat.y, position.x - boat.x))
      Ph = self.Gh * abs(previousDirection - angle)
      return Ph

  #Suma Potencjalow 
  def PotentialSum(self, boat, position, goal, obstacle, windDirection, previousDirection):
      GoalP = Potentials.PotentialGoal(position, goal)
      ObstacleP = Potentials.PotentialObstacle(position, obstacle)
      UpwindP = Potentials.PotentialUpwind(boat, position, windDirection)
      DownwindP = Potentials.PotentialDownwind(boat, position, windDirection)
      AngleChangeP = Potentials.PotentialAngleChange(boat, position, windDirection, previousDirection)
      Psum = GoalP + ObstacleP + UpwindP + DownwindP + AngleChangeP
      return Psum
    
class Server:
    def __init__(self):
        self.longitudePV = 10
        self.latitudePV = 10
        self.longitudeSP = 10
        self.latitudeSP = 10
        self.previousDirection = 0
        self.windDirection = 0
        
    def Data_callback(self, msg):
        self.longitudePV = msg.longitude/100
        self.latitudePV = msg.latitude/100
        self.previousDirection = msg.yaw
        self.windDirection = msg.angle

    def LongSP_callback(self, msg):
        self.longitudeSP = msg.data

    def LatiSP_callback(self, msg):
        self.latitudeSP = msg.data
        
    def loop(self):
        windDw= self.windDirection
        course = self.previousDirection
        boat = Point(self.longitudePV,self.latitudePV)
        goal = Point(self.longitudeSP,self.latitudeSP)
        obstacle = Point(60.959218,60.220071) #Deklaracja przeszkody poza obszarem
        
        if windDw > 180:
          windDw = - 360 + windDw
          
        windDbw = course - windDw
        x = windDbw

        if getDistanceInMeters(boat, goal) > 5: 
            radius = 0.00005  #rozmiar okregu na ktorym szukamy punktow 3,96m
            index = 0
            #liczenie potencjalu na okregu dla punktow co 2 stopnie
            for angle in range(0, 360, 2):
              x = boat.x + radius*math.cos(math.radians(angle))
              y = boat.y + radius*math.sin(math.radians(angle))
              position = Point(x, y)
              Potential = Potentials.PotentialSum(boat, position, goal, obstacle, windDbw, course)
              window[index] = Potential
              X[index] = x
              Y[index] = y
              index = index + 1

            minIndex = np.argmin(window) #indeks dla minimum na okregu
            #X[minIndex] pozadana zmiana pozycji statku na wyznaczone minimum
            #Y[minIndex] pozadana zmiana pozycji statku na wyznaczone minimum
            x = math.degrees(math.atan2(Y[minIndex]  - boat.y, X[minIndex]- boat.x))    
        
        pubcourse(x)

if __name__ == '__main__':
    rospy.init_node('PotientialField', anonymous=True)
    rate = rospy.Rate(0.2) # 1hz
    server = Server()
    
    rospy.Subscriber('realData', ShipData, server.Data_callback)
    rospy.Subscriber('LongitudeFromRED', Float64, server.LongSP_callback)
    rospy.Subscriber('LatitudeFromRED', Float64, server.LatiSP_callback)
    
    while not rospy.is_shutdown():
        server.loop()
        rate.sleep()
        
    rospy.spin()
