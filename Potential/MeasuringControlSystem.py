#!/usr/bin/env python

import rospy
import struct
import serial
import frameManagement as fr
from time import sleep
from std_msgs.msg import Int32
from std_msgs.msg import Bool


def readData(ser):
    bytesToRead = ser.inWaiting() #Sprawdzenie rozmiaru buffora
    #print(bytesToRead, "bajtow")
    if bytesToRead > 0:
        received_data = ser.read(bytesToRead) #Odczytanie danych z bufora
        #print(received_data)                       #print received data
        dataFormat = 'B' #Ustawienie formatu danych na unsigned char
        frameSize = str(bytesToRead) + dataFormat #ustawienie formatu i ilosci danych
        unpackedFrame = struct.unpack(frameSize,received_data) #odpakowanie
        translatedFrame = fr.translateInputFrame(unpackedFrame) #tlumaczenie (hex as string)
        #print(translateFrame)

        #verify input format and flags
        res = fr.validateInput(translatedFrame)
        if res != 1:
            print('flag error')
        else:
            #remove byte stuffing
            fr.removeFlags(translatedFrame)
            #verify length and CRC
            #print(translatedFrame)
            res = fr.verifyFrame(translatedFrame)
            if res != 1:
                print('crc8 error')
            else:
                fr.dispatchFrame(translatedFrame)

def writeData(serialport,frame):
    serialport.write(serial.to_bytes(frame))

class Server:
    def __init__(self):
        self.sail = None
        self.rudder = None
        self.sailM = None
        self.rudderM = None
        self.manual = 'False'
        self.OldSail = 50
        self.OldRudder = 50
        self.rudderF = 0
        self.sailF = 0

    def sail_callback(self, msg):
        # "Store" message received.
        self.sail = msg.data

    def rudder_callback(self, msg):
        # "Store" the message received.
        self.rudder = msg.data

    def ManualControl_callback(self, msg):
        # "Store" the message received.
        self.manual = msg.data

    def sailManual_callback(self, msg):
        # "Store" the message received.
        self.sailM = msg.data

    def rudderManual_callback(self, msg):
        # "Store" the message received.
        self.rudderM = msg.data

    def sailFuse_callback(self, msg):
        self.sailF = msg.data
        
    def rudderFuse_callback(self, msg):
        self.rudderF =  msg.data
        
    def setServo(self,serialport):
        print("Manual mode: ", self.manual)
        if self.manual == True:
            print("Manual")
            self.sail = self.sailM
            self.rudder = self.rudderM

        if self.sail == None or self.rudder == None:
            frame = fr.setServoFrame(50,50) #Servo1 sail, #Serwo0 rudder
            print("brak danych ustawiono Serwo: sail:", 50 ," rudder: ", 50)
            writeData(serialport,frame)
        else:        
            if self.sail == self.OldSail and self.rudder == self.OldRudder:
                print("brak  zmiany ustawienia Serwo: sail:",self.sail ," rudder: ",self.rudder)
            else:
                print("ustawiono Serwo: sail:",self.sail ," rudder: ",self.rudder)
                frame = fr.setServoFrame(self.sail, self.rudder)
                writeData(serialport,frame)
            
        self.OldSail = self.sail
        self.OldRudder = self.rudder


if __name__ == '__main__':
    print("Hello Statek")
    rospy.init_node('MeasuringControlSystem')
    rate = rospy.Rate(3) # 1hz
    ser = serial.Serial ("/dev/serial0", 115200, timeout=2)

    server = Server()
    rospy.Subscriber('ManualControl', Bool, server.ManualControl_callback)
    rospy.Subscriber('SetRudderManual', Int32, server.rudderManual_callback)
    rospy.Subscriber('SetSailManual', Int32, server.sailManual_callback)
    rospy.Subscriber('SetSail', Int32 , server.sail_callback)
    rospy.Subscriber('SetRudder', Int32, server.rudder_callback)
    rospy.Subscriber('sailFuse', Int32 , server.sailFuse_callback)
    rospy.Subscriber('rudderFuse', Int32, server.rudderFuse_callback)

    while not rospy.is_shutdown():
        frame = fr.reqBasicSensor()        #Utworzenie ramki zapytania
        writeData(ser,frame)  #Wyslanie zapytania
        print('Wyslano ramke reqBasicSensor')
        sleep(0.1)
        try:
            readData(ser)
        except rospy.ROSInterruptException:
            pass
        print('Odczytano dane')
        server.setServo(ser)
        sleep(0.1)
        try:
            readData(ser)
        except rospy.ROSInterruptException:
            pass
        rate.sleep()

    rospy.spin()
