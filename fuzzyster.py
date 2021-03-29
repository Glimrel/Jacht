#0 to maksymalne wychylenie na bakburtę, 100 to maksymalne wychylenie na sterburtę.
# e to błąd kursu
# de to pochodna błędu kursu * 10

#Przesłanki
eNL = [-40, -15]
eNS = [-20, -10, -3]
eZ = [-5, 0, 5]
ePS = [3, 10, 20]
ePL = [15, 40]

deNL = [-40, -15]
deNS = [-20, -10, -3]
deZ = [-5, 0, 5]
dePS = [3, 10, 20]
dePL = [15, 40]

#Konkluzje
MaxBakburta = [1, 2]
MocnoBakburta = [0,15,30]
LekkoBakburta = [25,35,45]
SterZero = [40,50,60]
LekkoSterburta = [55,65,75]
MocnoSterburta = [70,85,100]
MaxSterburta = [98, 99]

#Zdefiniowanie aktualnej wartości przesłanki
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
    fuzzyvalue = fuzzyAnd( eNLMF(error) , deNSMF(errorDerivative) )
    rules[1] = gettriMFConclusion(outputrange,MocnoSterburta,fuzzyvalue)
    # RULE 3
    fuzzyvalue = fuzzyAnd( eNLMF(error) , fuzzyOr( deZMF(errorDerivative) , dePSMF(errorDerivative) ) )
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
    fuzzyvalue = fuzzyAnd( ePLMF(error) , fuzzyOr( deNSMF(errorDerivative) , deZMF(errorDerivative) ) )
    rules[16] = gettriMFConclusion(outputrange,LekkoBakburta,fuzzyvalue)
    # RULE 18
    fuzzyvalue = fuzzyAnd( ePLMF(error) , dePSMF(errorDerivative) )
    rules[17] = gettriMFConclusion(outputrange,MocnoBakburta,fuzzyvalue)
    # RULE 19
    fuzzyvalue = fuzzyAnd( ePLMF(error) , dePLMF(errorDerivative) )
    rules[18] = getzMFConclusion(outputrange,MaxBakburta,fuzzyvalue)

    return rules

def deffuzyfication(rule):
    #Aggregacja reguł
    fuzzyoutput = AggregateConclusions(rule)
    #Wyostrzanie
    rudder = defuzzificationCentroid(fuzzyoutput)
    return rudder
