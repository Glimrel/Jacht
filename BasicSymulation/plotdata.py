#!/usr/bin/env python

import rospy
from itertools import count
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sandbox.msg import Objdata
from std_msgs.msg import Int32

# use ggplot style for more sophisticated visuals
plt.style.use('fivethirtyeight')

class Server:
    def __init__(self):
        self.index = count()
        self.sp = 0
        self.pv = 0
        self.x = 0
        self.y = 0
        self.cTime = []
        self.cPV = []
        self.cSP = []
        self.cX = []
        self.cY = []
        self.cXp = []
        self.cYp = []
        self.cTimep = []


    def spcallback(self, msg):
        self.sp = msg.data

    def datacallback(self, msg):
        self.pv = msg.course_angle
        self.x = msg.x
        self.y = msg.y

    def animate(self, i):
        self.cTime.append(next(self.index))
        self.cPV.append(self.pv)
        self.cSP.append(self.sp)

        if len(self.cTime) > 400:
            self.cTime.pop(0)
            self.cPV.pop(0)
            self.cSP.pop(0)

        fig1 = plt.subplot(2,1,1)
        fig1.cla()
        fig1.set_xlabel('n')
        fig1.set_ylabel('angle [degrees]')
        fig1.plot(self.cTime, self.cPV, self.cTime, self.cSP)

    def animate2(self, i):
        self.cX.append(self.x)
        self.cY.append(self.y)
        fig2 = plt.subplot(2,1,2)
        fig2.cla()

        if self.cTime:
            if self.cTime[len(self.cTime)-1] % 100 == 0:
                self.cXp.append(self.x)
                self.cYp.append(self.y)
                self.cTimep.append(self.cTime[len(self.cTime)-1])
        if self.cXp:
            for i in range(0,len(self.cXp)):
                fig2.text(self.cYp[i],self.cXp[i],'n=%i'%self.cTimep[i])

        fig2.set_xlabel('position x')
        fig2.set_ylabel('position y')
        fig2.plot(self.cY, self.cX)

    def ploting(self):
        ani = FuncAnimation(plt.gcf(), server.animate, 1000)
        ani2 = FuncAnimation(plt.gcf(), server.animate2, 1000)
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    rospy.init_node('Plot', anonymous=True)
    rate = rospy.Rate(10) # 1hz
    server = Server()

    rospy.Subscriber('symulationData', Objdata, server.datacallback)
    rospy.Subscriber('SP', Int32, server.spcallback)

    while not rospy.is_shutdown():
        server.ploting()
        rate.sleep()

    rospy.spin()