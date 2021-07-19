#!/usr/bin/env python

import rospy
from itertools import count
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sandbox.msg import Objdata
from std_msgs.msg import Float64,Int32

# use ggplot style for more sophisticated visuals
plt.style.use('fivethirtyeight')

class Server:
    def __init__(self):
        self.index = count()
        self.sp = 0
        self.pv = 0
        self.x = 100
        self.y = 100
        self.wind = 0
        self.cTime = []
        self.cPV = []
        self.cSP = []
        self.cX = []
        self.cY = []
        self.cXp = []
        self.cYp = []
        self.cTimep = []
        self.cWind = []
        self.cWindp = []


    def spcallback(self, msg):
        self.sp = msg.data
        
    def windcallback(self, msg):
        self.wind = msg.data

    def datacallback(self, msg):
        self.pv = msg.course_angle
        self.x = msg.x
        self.y = msg.y

    def animate(self, i):
        self.cTime.append(next(self.index))
        #self.cPV.append(self.pv)
        #self.cSP.append(self.sp)

        if len(self.cTime) > 400:
            self.cTime.pop(0)
            #self.cPV.pop(0)
            #self.cSP.pop(0)

        #fig1 = plt.subplot(3,1,1)
        #fig1.cla()
        #fig1.set_xlabel('n')
        #fig1.set_ylabel('angle [degrees]')
        #fig1.plot(self.cTime, self.cPV, self.cTime, self.cSP)

    def animate2(self, i):
        self.cX.append(self.x)
        self.cY.append(self.y)
        fig2 = plt.subplot(1,1,1)
        fig2.cla()

        if self.cTime:
            if self.cTime[len(self.cTime)-1] % 500 == 0:
                self.cXp.append(self.x)
                self.cYp.append(self.y)
                self.cTimep.append(self.cTime[len(self.cTime)-1])
                
        if self.cXp:
            for i in range(0,len(self.cXp)):
                fig2.text(self.cYp[i],self.cXp[i],'n=%i'%self.cTimep[i])

        fig2.set_xlabel('position x')
        fig2.set_ylabel('position y')
        fig2.plot(self.cY, self.cX)
        
    def animate3(self, i): 
        self.cWind.append(self.wind)
        fig3 = plt.subplot(3,1,3)
        fig3.cla()

        if self.cTime:
            if self.cTime[len(self.cTime)-1] % 100 == 0:
                self.cWindp.append(self.wind)
             
        if self.cWindp:
            for i in range(0,len(self.cWindp)):
                fig3.text(self.cTimep[i],self.cWindp[i],'n=%i'%self.cTimep[i])

        fig3.set_xlabel('n')
        fig3.set_ylabel('sail loose [%]')
        fig3.plot(self.cWind)

    def ploting(self):
        ani = FuncAnimation(plt.gcf(), server.animate, 1000)
        ani2 = FuncAnimation(plt.gcf(), server.animate2, 1000)
        #ani3 = FuncAnimation(plt.gcf(), server.animate3, 1000)
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    rospy.init_node('Plot', anonymous=True)
    rate = rospy.Rate(10) # 1hz
    server = Server()

    rospy.Subscriber('symulationData', Objdata, server.datacallback)
    rospy.Subscriber('setPoint', Float64, server.spcallback)
    rospy.Subscriber('SetSail', Int32, server.windcallback)

    while not rospy.is_shutdown():
        server.ploting()
        rate.sleep()

    rospy.spin()
