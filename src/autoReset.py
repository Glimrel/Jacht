#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool
from sandbox.msg import ShipData
from time import sleep

def reset():        
    print('Reset')
    GPIO.output(18, GPIO.LOW)
    sleep(0.01)
    print('Reset wykonano')
    GPIO.output(18, GPIO.HIGH)

class Server:
    def __init__(self):
        self.yaw = 0.1
        self.pitch = 0.1
        self.yawPOP = 1.1
        self.pitchPOP = 1.1
        self.yawPOP2 = 10.11
        self.pitchPOP2 = 10.11
        self.cnt = 0
        self.cnt2 = 0
        self.cnt3 = 0
        
    def datacallback(self, msg):
        self.yaw = msg.yaw
        self.pitch = msg.pitch
        Server.watchDog(self)

    def resetComand(self,msg): #Jesli rozkaz resetu 
        if msg.data == True:
            self.cnt = 0
            self.cnt2 = 0
            self.cnt3 = 0
            reset()
    
    def resetComand2(self,msg): #Jesli dwa razy MPU Fail
        if msg.data == True:
            self.cnt2 = self.cnt2 + 1
            print('Wykryto MPU Fail', self.cnt2)
        else:
            self.cnt2 = 0
            
        if self.cnt2 == 3:  
            reset()
	    self.cnt = 0
            self.cnt2 = 0
	    self.cnt3 = 0
    
    def watchDog(self): #Jesli dwa razy powtorza sie dane
        if self.yawPOP == self.yaw and self.pitchPOP == self.pitch:
            self.cnt = self.cnt + 1
            print('Wykryto MPU freeze', self.cnt)
        else:
            self.cnt = 0
        
        if self.cnt == 5:
            reset()
	    self.cnt = 0
	    self.cnt2 = 0
	    self.cnt3 = 0
        
        self.yawPOP = self.yaw
        self.pitchPOP = self.pitch

    def systemFail(self):
        if self.yawPOP2 == self.yaw and self.pitchPOP2 == self.pitch:
            self.cnt3 = self.cnt3 + 1
            print('Wykryto brak komunikacji z STM', self.cnt3)
        else:
            self.cnt3 = 0
        
        if self.cnt3 == 10:
            reset()
            self.cnt3 = 0
        
        self.yawPOP2 = self.yaw
        self.pitchPOP2 = self.pitch
        
    
if __name__ == '__main__':
    print("Start")
    rospy.init_node('Reset', anonymous=True)
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT)
    GPIO.output(18, GPIO.HIGH)
   
    server = Server()
    rospy.Subscriber('realData', ShipData, server.datacallback)
    rospy.Subscriber('reset', Bool, server.resetComand)
    rospy.Subscriber('reset2', Bool, server.resetComand2)  
    
    rate = rospy.Rate(0.5) # 1hz
    
    while not rospy.is_shutdown():
        server.systemFail()
        rate.sleep()
            
    rospy.spin()

