#!/usr/bin/env python

import rospy
import math
import numpy as np
import copy
from std_msgs.msg import Float64, Int32
from sandbox.msg import ShipData

def publish(rudder):
    publisher = rospy.Publisher('SetRudder', Int32, queue_size=10)
    publisher.publish(rudder)

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

def yawLimit(alfa):
  if alfa < -math.pi:
    alfa = 2 * math.pi + alfa
  if alfa > math.pi:
    alfa = - 2 * math.pi + alfa
  return alfa

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
  Xnew = distance.x * 0.00001 / 1.11 #Przesuniecie na ulamki sekund i skala
  Ynew = distance.y * 0.00001 / 1.11
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

def stanleyController(startPoint,goal,boat):
  #Obliczenie kata pozadanego
  yaw_ref = math.pi/2 - math.atan2(goal.y - startPoint.y, goal.x - startPoint.x) 
  yaw_ref = yawLimit(yaw_ref)
  yaw = math.pi/2 - math.atan2(goal.y - boat.y, goal.x - boat.x) 
  yaw = yawLimit(yaw)
  print('Kat zadany = ', math.degrees(yaw_ref), yaw_ref)
  print('Kat aktualny = ', math.degrees(yaw), yaw)
  #Roznica miedzy katem pozadanym, a rzeczywistym
  yaw_diff = ( yaw_ref - math.radians(yaw) ) 
  #print(yaw_diff)

  if yaw_diff > math.pi:
      yaw_diff = yaw_diff - 2 * math.pi
  elif yaw_diff < - math.pi:
      yaw_diff = yaw_diff + 2 * math.pi

  theta_deg = - math.degrees(yaw_diff) * 5 #Mozliwe ze trzeba odwrocic
  theta_deg = 50 + theta_deg #przeniesienie srodka na 50 z 0

  if theta_deg <= 0:
      theta_deg = 0

  if theta_deg >= 100:
      theta_deg = 100  

  return theta_deg

def debugPoint(Points):
  print('Wyswietlam punkty')
  for element in Points:
    print(element.x)
    print(element.y)


################################################################################
class Server:
    def __init__(self):
        self.longitudePV = 10
        self.latitudePV = 10
        self.longitudeSP = 10
        self.latitudeSP = 10
        self.yawM = 0
        self.windM = 0
        self.flag1 = 1
        self.flag2 = 1
        self.path = []
        self.StanleyPoints = []
        self.start = []
        self.StanleyGoal = []
        
    def Data_callback(self, msg):
        self.longitudePV = msg.longitude/100
        self.latitudePV = msg.latitude/100
        self.yawM = msg.yaw
        self.windM = msg.angle

    def LongSP_callback(self, msg):
        self.longitudeSP = msg.data
        self.flag1 = 1

    def LatiSP_callback(self, msg):
        self.latitudeSP = msg.data
        self.flag2 = 1
      
    def pathPlaner(self, boat, goal, yaw, windDirection):        
        #Parametry
        upWind = 44
        downWind = 20
        zwroty = 2
        
        #ALGORYTM
        #Obliczenie parametrow punktu docelowego
        distanse = getDistanceInMeters(boat,goal)
        directCourse = directCourseDeg(boat,goal)

        #Obliczenie kata wiatru dla kursu bezposredniego do celu
        globalWindDirection = yaw + windDirection 
        globalWindDirection = windLimit(globalWindDirection)
        windDirectionForDirectCourse = globalWindDirection - directCourse 
        windDirectionForDirectCourse = windLimit(windDirectionForDirectCourse)

        if abs(windDirectionForDirectCourse) < upWind: #Sprawdzenie warunkow pod wiatr
          Points = tackingAlgorithm(windDirectionForDirectCourse, distanse, zwroty, 45)
          Points = getPointsRotation(Points,directCourse) 
          Points = getGeoTransform(Points,boat)
        elif abs(abs(windDirectionForDirectCourse)-180) < downWind:  
          Points = tackingAlgorithm(windDirectionForDirectCourse, distanse, zwroty, 20)
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

        if getDistanceInMeters(boat, self.StanleyGoal) < 5: #Osiagniecie punktu docelowego
            if not self.StanleyPoints: #Przerwanie, dotarlismy do celu
              print('Osiagnieto CEL')
            else: #Nowy cel
              print('Obrano nowy CEL')
              self.start = self.StanleyGoal
              self.StanleyGoal = self.StanleyPoints.pop(0) 

        theta = stanleyController(self.start,self.StanleyGoal,boat)               
        publish(int(theta))
        print('Obliczona wartosc nastawy steru wynosi:', int(theta))
              
if __name__ == '__main__':
    rospy.init_node('Controller', anonymous=True)
    rate = rospy.Rate(0.2) # 1hz
    server = Server()
    
    rospy.Subscriber('realData', ShipData, server.Data_callback)
    rospy.Subscriber('LongitudeFromRED', Float64, server.LongSP_callback)
    rospy.Subscriber('LatitudeFromRED', Float64, server.LatiSP_callback)
    
    while not rospy.is_shutdown():
        server.loop()
        rate.sleep()
        
    rospy.spin()
