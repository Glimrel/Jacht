#!/usr/bin/env python

import rospy
import numpy as np
from sandbox.msg import Courseerror
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

#0 to maksymalne wychylenie na bakburte, 100 to maksymalne wychylenie na sterburte.
# e to blad kursu
# de to pochodna bledu kursu * 10

#Przeslanki
eNL = [-40, -15, 20]
eNS = [-20, -10, -3]
eZ = [-5, 0, 5]
ePS = [3, 10, 20]
ePL = [15, 40, 20]

deNL = [-40, -15, 20]
deNS = [-20, -10, -3]
deZ = [-5, 0, 5]
dePS = [3, 10, 20]
dePL = [15, 40, 20]

#Konkluzje
MaxBakburta = [1, 2, 1]
MocnoBakburta = [0,15,30]
LekkoBakburta = [25,35,45]
SterZero = [40,50,60]
LekkoSterburta = [55,65,75]
MocnoSterburta = [70,85,100]
MaxSterburta = [98, 99, 1]

#Zdefiniowanie aktualnej wartosci przeslanki
#Uchyb
def eNLMF(value):
    return zMF(value, eNL)

def eNSMF(value):
    return triMF(value, eNS)

def eZMF(value):
    return triMF(value, eZ)

def ePSMF(value):
    return triMF(value, ePS)

def ePLMF(value):
    return sMF(value, ePL)

#Pochodna uchybu
def deNLMF(value):
    return zMF(value, deNL)

def deNSMF(value):
    return triMF(value, deNS)

def deZMF(value):
    return triMF(value, deZ)

def dePSMF(value):
    return triMF(value, dePS)

def dePLMF(value):
    return sMF(value, dePL)

def FuzzyRules(error,errorDerivative):
    outputrange = 101
    numberofrules = 19
    rules = [[0]*outputrange]*numberofrules
    # RULE 1
    fuzzyvalue = fuzzyAnd( eNLMF(error) , deNLMF(errorDerivative) )
    rules[0] = getsMFConclusion(outputrange,MaxSterburta,fuzzyvalue)
    # RULE 2
    fuzzyvalue = fuzzyAnd( eNLMF(error) , fuzzyOr( deZMF(errorDerivative) , deNSMF(errorDerivative) ) )
    rules[1] = gettriMFConclusion(outputrange,MocnoSterburta,fuzzyvalue)
    # RULE 3
    fuzzyvalue = fuzzyAnd( eNLMF(error) , dePSMF(errorDerivative) )
    rules[2] = gettriMFConclusion(outputrange,LekkoSterburta,fuzzyvalue)
    # RULE 4
    fuzzyvalue = fuzzyAnd( eNLMF(error) , dePLMF(errorDerivative) )
    rules[3] = gettriMFConclusion(outputrange,SterZero,fuzzyvalue)

    # RULE 5
    fuzzyvalue = fuzzyAnd( eNSMF(error) , deNLMF(errorDerivative) )
    rules[4] = gettriMFConclusion(outputrange,MocnoSterburta,fuzzyvalue)
    # RULE 6
    fuzzyvalue = fuzzyAnd( eNSMF(error) , fuzzyOr( deNSMF(errorDerivative) , deZMF(errorDerivative) ) )
    rules[5] = gettriMFConclusion(outputrange,LekkoSterburta,fuzzyvalue)
    # RULE 7
    fuzzyvalue = fuzzyAnd( eNSMF(error) , dePSMF(errorDerivative) )
    rules[6] = gettriMFConclusion(outputrange,SterZero,fuzzyvalue)
    # RULE 8
    fuzzyvalue = fuzzyAnd( eNSMF(error) , dePLMF(errorDerivative) )
    rules[7] = gettriMFConclusion(outputrange,LekkoBakburta,fuzzyvalue)

    # RULE 9
    fuzzyvalue = fuzzyAnd( eZMF(error) , fuzzyOr( deNLMF(errorDerivative) , deNSMF(errorDerivative) ) )
    rules[8] = gettriMFConclusion(outputrange,LekkoSterburta,fuzzyvalue)
    # RULE 10
    fuzzyvalue = fuzzyAnd( eZMF(error) , deZMF(errorDerivative) )
    rules[9] = gettriMFConclusion(outputrange,SterZero,fuzzyvalue)
    # RULE 11
    fuzzyvalue = fuzzyAnd( eZMF(error) , fuzzyOr( dePSMF(errorDerivative) , dePLMF(errorDerivative) ) )
    rules[10] = gettriMFConclusion(outputrange,LekkoBakburta,fuzzyvalue)

    # RULE 12
    fuzzyvalue = fuzzyAnd( ePSMF(error) , deNLMF(errorDerivative) )
    rules[11] = gettriMFConclusion(outputrange,LekkoSterburta,fuzzyvalue)
    # RULE 13
    fuzzyvalue = fuzzyAnd( ePSMF(error) , deNSMF(errorDerivative) )
    rules[12] = gettriMFConclusion(outputrange,SterZero,fuzzyvalue)
    # RULE 14
    fuzzyvalue = fuzzyAnd( ePSMF(error) , fuzzyOr( deZMF(errorDerivative) , dePSMF(errorDerivative) ) )
    rules[13] = gettriMFConclusion(outputrange,LekkoBakburta,fuzzyvalue)
    # RULE 15
    fuzzyvalue = fuzzyAnd( ePSMF(error) , dePLMF(errorDerivative) )
    rules[14] = gettriMFConclusion(outputrange,MocnoBakburta,fuzzyvalue)

    # RULE 16
    fuzzyvalue = fuzzyAnd( ePLMF(error) , deNLMF(errorDerivative) )
    rules[15] = gettriMFConclusion(outputrange,SterZero,fuzzyvalue)
    # RULE 17
    fuzzyvalue = fuzzyAnd( ePLMF(error) , deNSMF(errorDerivative) )
    rules[16] = gettriMFConclusion(outputrange,LekkoBakburta,fuzzyvalue)
    # RULE 18
    fuzzyvalue = fuzzyAnd( ePLMF(error) , fuzzyOr( dePSMF(errorDerivative) , deZMF(errorDerivative) ) )
    rules[17] = gettriMFConclusion(outputrange,MocnoBakburta,fuzzyvalue)
    # RULE 19
    fuzzyvalue = fuzzyAnd( ePLMF(error) , dePLMF(errorDerivative) )
    rules[18] = getzMFConclusion(outputrange,MaxBakburta,fuzzyvalue)

    return rules

def deffuzyfication(rule):
    #Aggregacja regul
    fuzzyoutput = AggregateConclusions(rule)
    #Wyostrzanie
    rudder = defuzzificationCentroid(fuzzyoutput)
    return rudder

def publish(rudder):
    publisher = rospy.Publisher('SetRudder', Int32, queue_size=10)
    publisher.publish(rudder)

def callback(data):
    error = data.error
    errorDerivative = data.errorDerivative
    rule = FuzzyRules(error,errorDerivative)
    rudder = deffuzyfication(rule)
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    publish(rudder)
    print('Obliczona wartosc nastawy steru wynosi:', rudder)

def listener():

    rospy.init_node('fuzzyRudderControler', anonymous=True)

    rospy.Subscriber('CourseFuzzyHub', Courseerror, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
