#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Int32

# S-ksztaltna funkcja przynaleznosci
def sMF(x,points):
  leftpoint = points[0]
  rightpoint = points[1]
  range = points[2]
  b = (leftpoint + rightpoint)/2
  a = range/(2*(rightpoint - leftpoint))
  xfuzzy =  100/(1 + np.exp(a * (-x + b)))
  return xfuzzy   

# Z-ksztaltna funkcja przynaleznosci
def zMF(x,points):
  leftpoint = points[0]
  rightpoint = points[1]
  range = points[2]
  b = (leftpoint + rightpoint)/2
  a = range/(2*(rightpoint - leftpoint))
  xfuzzy =  100/(1 + np.exp(a * (x - b)))
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

#Trapezoidalna funkcja przynaleznosci
def trapMF(x, points):
    leftpoint = points[0]
    leftcenterpoint = points[1]
    rightcenterpoint = points[2]
    rightpoint = points[3]

    if x <= leftpoint or x >= rightpoint:
      xfuzzy = 0
    elif x <= leftcenterpoint:
      xfuzzy = ((x - leftpoint)*100) / (leftcenterpoint - leftpoint)
    elif x <= rightcenterpoint:
      xfuzzy = 100
    elif x < rightpoint:
      xfuzzy = ((rightpoint - x)*100) / (rightpoint - rightcenterpoint)
    return xfuzzy

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

#Trojkatna funkcja konkluzji
def gettriMFConclusion(outputrange,points,fuzzyvalue):
  conclusionPlot = [0] * outputrange
  for i in range(outputrange):
    conclusionPlot[i]= min(triMF(i,points),fuzzyvalue)
  return conclusionPlot

#Trapezoidalna funkcja konkluzji
def gettrapMFConclusion(outputrange,points,fuzzyvalue):
  conclusionPlot = [0] * outputrange
  for i in range(outputrange):
    conclusionPlot[i]= min(trapMF(i,points),fuzzyvalue)
  return conclusionPlot

#S-ksztaltna funkcja konkluzji
def getsMFConclusion(outputrange,points,fuzzyvalue):
  conclusionPlot = [0] * outputrange
  for i in range(outputrange):
    conclusionPlot[i]= min(sMF(i,points),fuzzyvalue)
  return conclusionPlot

#Z-ksztaltna funkcja konkluzji
def getzMFConclusion(outputrange,points,fuzzyvalue):
  conclusionPlot = [0] * outputrange
  for i in range(outputrange):
    conclusionPlot[i]= min(zMF(i,points),fuzzyvalue)
  return conclusionPlot

#Agregacja koknluzji
def AggregateConclusions(conclusions):
  outputrange = len(conclusions[0])
  numberofconclusions= len(conclusions)
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

# Rozmyty uklad ustawienia zagla
# 0 - max wybrany
# 100 - max zluzowany

#Przeslanki
Fordewind = [150, 180, 20]
Baksztag = [105, 135, 165]
Polwiatr = [60, 90, 120]
Bajdewind = [45, 60 ,85]
Martwy = [20,60, 20]

#Konkluzje
Wyluzowane = [75, 110, 26];
Wybrane = [50, 75, 100]
MocnoWybrane =[25, 50, 75]
BardzoMocnoWybrane =[0, 15, 30]
MaksymalnieWybrane = [-5, 20, 20]

def FordewindMF(WindDirection):
    return sMF(WindDirection, Fordewind)

def BaksztagMF(WindDirection):
    return triMF(WindDirection, Baksztag)

def PolwiatrMF(WindDirection):
    return triMF(WindDirection, Polwiatr)

def BajdewindMF(WindDirection):
    return triMF(WindDirection, Bajdewind)

def MartwyMF(WindDirection):
    return zMF(WindDirection, Martwy)

def FuzzyRules(WindDirection):
    outputrange = 113
    numberofrules = 5
    rules = [[0]*outputrange]*numberofrules
    # RULE 1
    fuzzyvalue = FordewindMF(WindDirection)
    rules[0] = getsMFConclusion(outputrange,Wyluzowane,fuzzyvalue)

    # RULE 2
    fuzzyvalue = BaksztagMF(WindDirection)
    rules[1] =gettriMFConclusion(outputrange,Wybrane,fuzzyvalue)

    # RULE 3
    fuzzyvalue = PolwiatrMF(WindDirection)
    rules[2] =gettriMFConclusion(outputrange,MocnoWybrane,fuzzyvalue)

    # RULE 4
    fuzzyvalue = BajdewindMF(WindDirection)
    rules[3] = gettriMFConclusion(outputrange,BardzoMocnoWybrane,fuzzyvalue)

    # RULE 5
    fuzzyvalue = MartwyMF(WindDirection)
    rules[4] = getzMFConclusion(outputrange,MaksymalnieWybrane,fuzzyvalue)
    return rules

def deffuzyfication(rule):
    #Aggregacja regul
    fuzzyoutput = AggregateConclusions(rule)
    #Wyostrzanie
    sail = defuzzificationCentroid(fuzzyoutput)
    return sail

def publish(sail):
    publisher = rospy.Publisher('SetSail', Int32, queue_size=10)
    publisher.publish(sail)

def callback(data):
    WindDirection = data.data
    rule = FuzzyRules(WindDirection)
    sail = deffuzyfication(rule)
    
    print("Dla kierunku", WindDirection, "kat zagla wynosi: ", sail)
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    publish(sail)


def listener():

    rospy.init_node('fuzzySailControler', anonymous=True)

    rospy.Subscriber('SailFuzzyHub', Int32, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
