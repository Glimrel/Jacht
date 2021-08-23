#!/usr/bin/env python

import rospy
import math
import numpy as np
import copy
from std_msgs.msg import Float64, Int32
from sandbox.msg import Objdata

def pubSP(msg):
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

#Algorytm halsu
def tackingAlgorithm(Wind, Distanse, zwroty, windType):
    #Obliczenie katow halsu
    alfa = windType + Wind
    beta = windType - Wind
    sum = 0
    heuristic = 1 #bazowa heurystyka dla 45 stopni

    while abs(sum - Distanse) > 1: #petla aktualizacji heurystyki
      BaseLength = heuristic * 0.2 * Distanse #Obliczenie podstawy prostokata

      FirstTack = 1/2 * BaseLength / math.cos(math.radians(90-alfa)) #Obliczenie dlugosci trasy halsem
      StarboartTack =  BaseLength / math.cos(math.radians(90-beta))
      PortTack = BaseLength / math.cos(math.radians(90-alfa))
      LastTack = 1/2 * BaseLength / math.cos(math.radians(90-beta))

      PlannedDistanceFirst = FirstTack * math.sin(math.radians(90-alfa)) #Obliczenie przebytego dystansu do celu
      PlannedDistance2 = StarboartTack * math.sin(math.radians(90-beta))
      PlannedDistance3 = PortTack * math.sin(math.radians(90-alfa))
      PlannedDistanceLast = LastTack * math.sin(math.radians(90-beta))
      Tack = [PlannedDistance2, PlannedDistance3]

      sum = PlannedDistanceFirst + zwroty/2 * Tack[0] + zwroty/2 * Tack[1] + PlannedDistanceLast #Suma przebytego dystansu
      heuristic = Distanse / sum  #Aktualizacja heurystyki

    # Wyznaczenie punktow zwrotu
    list = [] 
    list.append(Point(0,0))
    list.append(Point(BaseLength, PlannedDistanceFirst))

    znak = -1
    Distance = PlannedDistanceFirst
    for i in range(1,zwroty+1):
      m = (i+1)%2
      Distance = Distance + Tack[m]
      list.append(Point (znak * BaseLength, Distance))
      znak = znak * -1

    list.append(Point(0, sum))

    return list

#Limity wiatru
def windLimit(wind):
  if wind < -180:
    wind = 360 + wind
  if wind > 180:
    wind = - 360 + wind
  return wind

#Lista dla dwoch punktow
def directPoints(position, goal):
  list = [] 
  list.append(position)
  list.append(goal)

  return list

#Obliczenie kursu bezposredniego radiany
def directCourseRad(position, goal):
  K = math.pi/2 - math.atan2(goal.y - position.y, goal.x - position.x) 
  return K

#Obliczenie kursu bezposredniego stopnie
def directCourseDeg(position, goal):
  K = math.pi/2 - math.atan2(goal.y - position.y, goal.x - position.x) 
  K = math.degrees(K)
  return K 

#Obliczenie dystansu w metrach na podstawie dwoch punktow geografcznych
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

#Obliczenie wspolrzednych geograficznych dla odleglosci w metrach
def getGeoPoint(point, distance):
  Xnew = distance.x
  Ynew = distance.y 
  newPoint = Point(point.x + Xnew, point.y + Ynew)

  return newPoint

#Transformacje przestrzeni punktow w metrach na punkty geolokacyjne
def getGeoTransform(Points,position):
  list = [] 
  first = Points.pop(0)
  firstGeo = Point(first.x + position.x, first.y + position.y)
  list.append(firstGeo)

  while Points:
    next = Points.pop(0)
    nextGeo = getGeoPoint(firstGeo, next)
    list.append(nextGeo)

  return list

#Rotacja uzyskanego halsu dla zadanego kursu
def getPointsRotation(points, yaw):
  list = [] 
  alfa = math.radians(-yaw)
  rotationMatrix = np.array( [[ math.cos(alfa), -math.sin(alfa) ],[ math.sin(alfa), math.cos(alfa)]] )

  for point in points:
    xNew = math.cos(alfa) * point.x - math.sin(alfa) * point.y
    yNew = math.sin(alfa) * point.x + math.cos(alfa) * point.y
    newPoint = Point(xNew,yNew)
    list.append(newPoint)
  return list

def stanleyController(goal,start,boat,yaw):
  Kp = 2
  K = 2
    
  #Kurs wzgledem celu
  yaw_c = math.pi/2 - math.atan2(goal.y - boat.x, goal.x - boat.y)
  print('Kurs wzgledem celu yaw_c = ', math.degrees(yaw_c), yaw_c)
  #Kurs pozadany
  yaw_ref = math.pi/2 - math.atan2(goal.y - start.y, goal.x - start.x)
  print('Kurs pozadany yaw_ref = ', math.degrees(yaw_ref), yaw_ref)
  #Kurs aktualny
  yaw = math.radians(yaw)
  print('Kurs aktualny yaw = ', math.degrees(yaw), yaw)
  
  error = yaw_c - yaw_ref
  print('Error = ', error)
  
  if abs(error) > math.radians(K):
    yaw_ref_p = yaw_ref + Kp * error
  else:
    yaw_ref_p = yaw_c
    
  print('yaw_ref_p = ', yaw_ref_p )
  
  yaw_ref_p_deg = math.degrees(yaw_ref_p)
  
  if yaw_ref_p_deg > 180:
      yaw_ref_p_deg = yaw_ref_p_deg - 360
      
  if yaw_ref_p_deg < -180:
      yaw_ref_p_deg = yaw_ref_p_deg + 360
  
  pubSP(yaw_ref_p_deg)
  print('Kurs zadany yaw_ref_p = ', yaw_ref_p_deg)

################################################################################
class Server:
    def __init__(self):
        self.longitudePV = 100
        self.latitudePV = 100
        self.longitudeSP = 100
        self.latitudeSP = 100
        self.yawM = 0
        self.windM = 45
        self.flag1 = 1
        self.flag2 = 1
        self.path = []
        self.StanleyPoints = []
        self.start = []
        self.StanleyGoal = []
        
    def Data_callback(self, msg):
        self.longitudePV = msg.x
        self.latitudePV = msg.y
        self.yawM = msg.course_angle
        self.windM = msg.wind_direction

    def LongSP_callback(self, msg):
        self.flag1 = 1
        
        if msg.data == self.longitudeSP:
            self.flag1 = 0
            
        self.longitudeSP = msg.data

    def LatiSP_callback(self, msg):
        self.flag2 = 1
        
        if msg.data == self.latitudeSP:
            self.flag2 = 0
        
        self.latitudeSP = msg.data
        
    def pathPlaner(self, boat, goal, yaw, windDirection):        
        #Parametry
        upWind = 44
        downWind = 20
        zwroty = 2
        
        #ALGORYTM
        #Obliczenie parametrow punktu docelowego
        distanse = dist(boat,goal)
        directCourse = directCourseDeg(boat,goal)

        #Obliczenie kata wiatru dla kursu bezposredniego do celu
        globalWindDirection = windDirection 
        globalWindDirection = windLimit(globalWindDirection)
        windDirectionForDirectCourse = globalWindDirection - directCourse 
        windDirectionForDirectCourse = windLimit(windDirectionForDirectCourse)
        print('Kierunek wiatru: ', windDirection)
        print('Kurs bezposredni: ', directCourse)
        print('Wiatr dla kursu: ', windDirectionForDirectCourse)
        
        if abs(windDirectionForDirectCourse) < upWind: #Sprawdzenie warunkow pod wiatr
          Points = tackingAlgorithm(windDirectionForDirectCourse, distanse, zwroty, 30)
          Points = getPointsRotation(Points,directCourse) 
          Points = getGeoTransform(Points,boat)
        elif abs(abs(windDirectionForDirectCourse)-180) < downWind:  
          Points = tackingAlgorithm(windDirectionForDirectCourse, distanse, zwroty, 10)
          Points = getPointsRotation(Points,directCourse)
          Points = getGeoTransform(Points,boat)
        else: #Kurs bezposredni
          Points = directPoints(boat, goal)
        return Points
        
    def loop(self):
        #Inicjalizacja algorytmu
        boat = Point(self.longitudePV, self.latitudePV) #from GPS
        goal = Point(self.longitudeSP, self.latitudeSP) #from Node-RED
        yaw = self.yawM
        windDirection = self.windM
        
        if yaw > 180:
            yaw =  yaw - 360 
        
        if self.flag1 == 1 and self.flag2 == 1:
            print('Zmiana trasy')
            self.flag1 = 0
            self.flag2 = 0
            self.path = self.pathPlaner(boat, goal, yaw, windDirection)
            self.StanleyPoints = copy.copy(self.path)
            self.start = []
                   
        if not self.start:
            self.start = self.StanleyPoints.pop(0)
            self.StanleyGoal = self.StanleyPoints.pop(0)
        
        rev = Point(self.StanleyGoal.y, self.StanleyGoal.x)
        print('**********************')
        print('Odleglosc od celu = ', dist(boat, rev))
        if dist(boat, rev) < 20: #Osiagniecie punktu docelowego
            if not self.StanleyPoints: #Przerwanie, dotarlismy do celu
              print('Osiagnieto CEL')
            else: #Nowy cel
              print('Obrano nowy CEL')
              self.start = self.StanleyGoal
              self.StanleyGoal = self.StanleyPoints.pop(0) 

        print('Aktualny cel', self.StanleyGoal.y, self.StanleyGoal.x)
        print('')
        stanleyController(self.StanleyGoal,self.start,boat,yaw)
        print('')
        print('**********************')
        print('')
        print('')
              
if __name__ == '__main__':
    rospy.init_node('Controller', anonymous=True)
    rate = rospy.Rate(0.5) # 1hz
    server = Server()
    
    rospy.Subscriber('symulationData', Objdata, server.Data_callback)
    rospy.Subscriber('LongitudeFromRED', Float64, server.LongSP_callback)
    rospy.Subscriber('LatitudeFromRED', Float64, server.LatiSP_callback)
    
    while not rospy.is_shutdown():
        server.loop()
        rate.sleep()
        
    rospy.spin()


