#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from sandbox.msg import ShipData

def rudderPub(rudder):
    publisher = rospy.Publisher('rudderFuse', Int32, queue_size=10)
    publisher.publish(rudder)

def sailPub(sail):
    publisher = rospy.Publisher('sailFuse', Int32, queue_size=10)
    publisher.publish(sail)


# S-ksztaltna funkcja przynaleznosci
def sMF(x,points):
  leftpoint = points[0]
  rightpoint = points[1]

  if x <= leftpoint:
    xfuzzy = 0
  elif x < rightpoint:
    xfuzzy = ((x - leftpoint)*100) / (rightpoint - leftpoint)
  elif x >= rightpoint:
    xfuzzy = 100
  return xfuzzy

# Z-ksztaltna funkcja przynaleznosci
def zMF(x,points):  
  leftpoint = points[0]
  rightpoint = points[1]

  if x <= leftpoint:
    xfuzzy = 100
  elif x < rightpoint:
    xfuzzy = ((rightpoint - x)*100) / (rightpoint - leftpoint)
  elif x >= rightpoint:
    xfuzzy = 0
  return xfuzzy

# Trojkatna funkcja przynaleznocci
def triMF(x, points):
  leftpoint = points[0]
  centerpoint = points[1]
  rightpoint = points[2]

  if x <= leftpoint or x >= rightpoint:
    xfuzzy = 0
  elif x <= centerpoint:
    xfuzzy = ((x - leftpoint)*100) / (centerpoint - leftpoint)
  elif x < rightpoint:
    xfuzzy = ((rightpoint - x)*100) / (rightpoint - centerpoint)
  return xfuzzy

#Trojkatna funkcja konkluzji
def gettriMFConclusion(outputrange,points,fuzzyvalue):
  conclusionPlot = [0] * len(outputrange)
  iter = 0
  for i in outputrange:
    iter = iter + 1
    conclusionPlot[i]= min(triMF(i,points),fuzzyvalue)
  return conclusionPlot

#Singletonowa funkcja konkluzji
def getSingleMFConclusion(outputrange):
  conclusionPlot = [0] * len(outputrange)
  iter = 0
  for i in outputrange:
    if i == 0:
      conclusionPlot[iter]= 100
    else:
      conclusionPlot[iter]= 0
    iter = iter + 1
    
  return conclusionPlot

#Wyostrzanie metoda Centroid
def defuzzificationCentroid(fuzzyoutput):
  num = 0
  den = 0
  for i in range(len(fuzzyoutput)):
    num += i * fuzzyoutput[i]
  for i in range(len(fuzzyoutput)):
    den += fuzzyoutput[i]
  result = num / den
  return result

#Agregacja koknluzji
def AggregateConclusions(conclusions):
  outputrange = len(conclusions[0])
  fuzzyoutput = [0] * outputrange
  numberofconclusions= len(conclusions)
  comparearray = [0] * numberofconclusions

  for i in range(outputrange):
    for j in range(numberofconclusions):
      comparearray[j] = conclusions[j][i]
    fuzzyoutput[i]=max(comparearray)

  return fuzzyoutput

#Operator and - min
def fuzzyAnd(ruleA,ruleB):
  result = min(ruleA,ruleB)
  return result

#Operator or - max
def fuzzyOr(ruleA,ruleB):
  result = max(ruleA,ruleB)
  return result

#Przeslanki
Safe = [0, 30]
Risky = [20, 30, 40]
Dangerous  = [35,50] 

WindPort = [0,10]
WindStarboard = [-10,0]


#Konkluzje
RudderZero = 0
SailZero = 0
CounterRudderPortLight = [0,5,10] 
CounterRudderPortHard = [5,15,25] 
CounterRudderStarboardLight = [-10,-5,0] 
CounterRudderStarboardHard = [-25,-15,-5] 
CounterSail = [0,10,20]

def SafeMF(Roll):
    return zMF(Roll, Safe)

def RiskyMF(Roll):
    return triMF(Roll, Risky)

def DangerousMF(Roll):
    return sMF(Roll, Dangerous)

def WindPortMF(wind):
    return sMF(wind, WindPort)

def WindStarboardMF(wind):
    return zMF(wind, WindStarboard)


def FuzzyRulesSail(roll,wind):
    outputrange = range(-30,31)
    numberofrules = 2
    rules = [[0]*len(outputrange)]*numberofrules
    # RULE 1
    fuzzyvalue = SafeMF(roll)
    rules[0] = getSingleMFConclusion(outputrange)

    # RULE 2
    fuzzyvalue = DangerousMF(roll)
    rules[1] = gettriMFConclusion(outputrange, CounterSail, fuzzyvalue)

    return rules

def FuzzyRulesRudder(roll, wind):
  outputrange = range(-30,31)
  numberofrules = 5
  rules = [[0]*len(outputrange)]*numberofrules

  # RULE 1
  fuzzyvalue = SafeMF(roll)
  rules[0] = getSingleMFConclusion(outputrange)

  # RULE 2
  fuzzyvalue = fuzzyAnd(RiskyMF(roll), WindPortMF(wind))
  rules[1] = gettriMFConclusion(outputrange, CounterRudderStarboardLight, fuzzyvalue)

  # RULE 3
  fuzzyvalue = fuzzyAnd(RiskyMF(roll), WindStarboardMF(wind))
  rules[2] = gettriMFConclusion(outputrange,  CounterRudderPortLight, fuzzyvalue)

  # RULE 4
  fuzzyvalue = fuzzyAnd(DangerousMF(roll), WindPortMF(wind))
  rules[3] = gettriMFConclusion(outputrange, CounterRudderStarboardHard, fuzzyvalue)

  # RULE 5
  fuzzyvalue = fuzzyAnd(DangerousMF(roll), WindStarboardMF(wind))
  rules[4] = gettriMFConclusion(outputrange,  CounterRudderPortHard, fuzzyvalue)

  return rules
  

def deffuzyfication(rule):
  #Aggregacja regul
  fuzzyoutput = AggregateConclusions(rule) 
  #Wyostrzanie
  value = defuzzificationCentroid(fuzzyoutput) - 30
  return value

def callback(data):
    wind = data.angle
    roll = data.roll

    
    ruleRudder = FuzzyRulesRudder(roll, wind)
    ruleSail = FuzzyRulesSail(roll, wind)
    rudderChange = deffuzyfication(ruleRudder)
    sailChange = deffuzyfication(ruleSail)
    
    print("Zmiana ustawienia steru: ", rudderChange, "zmiana ustawienia zagla: ", sailChange)
    rudderPub(int(rudderChange))
    sailPub(int(sailChange))

    
def listener():
    rospy.init_node('Fuse', anonymous=True)
    rospy.Subscriber('realData', ShipData, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

