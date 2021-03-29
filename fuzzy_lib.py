#S-kształtna funkcja przynależności
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

#Z-kształtna funkcja przynależności
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

#Trójkątna funkcja przynależności
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

#Trapezoidalna funkcja przynależności
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

#Wyostrzanie metodą Centroid
def defuzzificationCentroid(fuzzyoutput):
  num = 0
  den = 0
  for i in range(len(fuzzyoutput)):
    num += i * fuzzyoutput[i]
  for i in range(len(fuzzyoutput)):
    den += fuzzyoutput[i]
  result = num / den
  return result

#Trójkątna funkcja konkluzji
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

#S-kształtna funkcja konkluzji
def getsMFConclusion(outputrange,points,fuzzyvalue):
  conclusionPlot = [0] * outputrange
  for i in range(outputrange):
    conclusionPlot[i]= min(sMF(i,points),fuzzyvalue)
  return conclusionPlot

#Z-kształtna funkcja konkluzji
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
