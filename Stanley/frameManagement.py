import rospy
import crc8
import struct
from sandbox.msg import ShipData

#constants
START_FLAG = 'E2'
END_FLAG = 'E3'
ESCAPE_SIGN = 'F5'

FRAMETYPE_ACK = '10'
FRAMETYPE_BASIC_SENSOR = '11'

FRAMETYPE_REQ_BASIC_SENSOR = '60'
FRAMETYPE_SET_SERVOS = '61'


#global variables
flags = [START_FLAG, END_FLAG, ESCAPE_SIGN]


#Function definitions

#Check if given byte is a special flag
#Return 1 if true, 0 if false
def isFlag(char):
    if char in flags:
        return 1
    else:
        return 0


#Validate input string - hex format and correct start and end flags
#Return 1 if valid, 0 if invalid
def validateInput(inString):

    error = 0

    for i in inString:
        #check bytes len
        if len(i) != 2:
            print('input error - all bytes must be given as 2 ASCII chars')
            error = 1
            break;
        #check if input is hex
        try:
            temp = int(i, 16)
        except ValueError:
            print('input error - only 0-9 and A-F chars allowed')
            error = 1
            break;

    first = inString[0]
    last = inString[len(inString) - 1]

    if first != "E2":
        print('input error - incorrect start flag 0x{}. Should be 0xE2'.format(first))
        error = 1
    if last != "E3":
        print('input error - incorrect end flag 0x{}. Should be 0xE3'.format(last))
        error = 1

    if error == 1:
        return 0
    else:
        return 1


#Remove special signs from frame
def removeFlags(frame):
    i = 0
    while i < len(frame):
        if frame[i] == ESCAPE_SIGN:
            if isFlag(frame[i+1]):  #escape sign before special sign - remove it
                del frame[i]
                i = i + 1
        i = i + 1

    #remove start and end flags
    del frame[0]
    del frame[len(frame) - 1]


#verify frame length and CRC
def verifyFrame(frame):

    realFrameLen = len(frame)
    declaredFrameLen = int(frame[1], 16)

    #verify frame length
    if realFrameLen != (declaredFrameLen + 3):
        print('Payload len {} not equal to declared in length field {}'.format(realFrameLen-3, declaredFrameLen))
        return 0

    #verify CRC
    declaredCRC = frame.pop(realFrameLen-1)
    hash = crc8.crc8()
    for i in frame:
        hash.update(str(bytearray.fromhex(i)))

    calcCRC = (hash.hexdigest()).upper()

    if calcCRC != declaredCRC:
        print("Calculated CRC 0x{} not equal to received 0x{}".format(calcCRC, declaredCRC))
        return 0

    return 1

#Check frame type and parse it with appropriate function
def dispatchFrame(frame):

    frameType = frame[0]
    if(len(frame) > 2):
        framePayload = frame[2:(len(frame))]  #omit frame type and len bytes

    if frameType == FRAMETYPE_ACK:
        print('ACK frame')

    elif frameType == FRAMETYPE_BASIC_SENSOR:
        print('Basic sensor data frame:')
        parseBasicSensor(framePayload)

    elif frameType == FRAMETYPE_REQ_BASIC_SENSOR:
        print('Request basic sensor data frame')

    elif frameType == FRAMETYPE_SET_SERVOS:
        print('Set servos frame:')
        parseSetServos(framePayload)

#Parse basic sensor data frame
def parseBasicSensor(frame):

    #convert strings to byte arrays
    Yaw = bytearray.fromhex(''.join(frame[0:4]))
    Pitch = bytearray.fromhex(''.join(frame[4:8]))
    Roll = bytearray.fromhex(''.join(frame[8:12]))
    Longitude = bytearray.fromhex(''.join(frame[12:16]))
    Latitude = bytearray.fromhex(''.join(frame[16:20]))
    Encoder = bytearray.fromhex(''.join(frame[20:24]))
    infoMask = bytearray.fromhex(frame[24])

    #convert byte arrays to floats
    YawFloat = struct.unpack('f', Yaw)
    PitchFloat = struct.unpack('f', Pitch)
    RollFloat = struct.unpack('f', Roll)
    LongitudeFloat = struct.unpack('f', Longitude)
    LatitudeFloat = struct.unpack('f', Latitude)
    EncoderFloat = struct.unpack('f', Encoder)

    LongitudeSign = 'N' if ( infoMask[0] & 0x01 ) else 'S'
    LatitudeSign = 'W' if ( infoMask[0] & 0x02 ) else 'E'
    MPURes = 'OK' if ( infoMask[0] & 0x04 ) else 'Fail'
    GPSRes = 'OK' if ( infoMask[0] & 0x08 ) else 'Fail'
    EncoderRes = 'OK' if ( infoMask[0] & 0x10 ) else 'Fail'

    print('9DoF {} : Yaw {:.3f}, Pitch {:.3f}, Roll {:.3f}'.format(MPURes, YawFloat[0], PitchFloat[0], RollFloat[0]))
    print('GPS {} : Longitude {:.3f}{}, Latitude {:.3f}{}'.format(GPSRes, LongitudeFloat[0], LongitudeSign, LatitudeFloat[0], LatitudeSign))
    print('Encoder {} : Angle {:.3f}'.format(EncoderRes, EncoderFloat[0]))

    #Zakres roll
    if RollFloat[0] > 0:
        msg.roll = RollFloat[0] - 180
    else:
        msg.roll = RollFloat[0] + 180
    
    #Utworzenie wiadomosci typu Object data
    msg = ShipData()
    msg.yaw = YawFloat[0]
    #msg.roll = RollFloat[0]
    msg.pitch = PitchFloat[0]
    msg.longitude = LongitudeFloat[0]
    msg.latitude = LatitudeFloat[0]
    msg.angle = EncoderFloat[0]
    msg.LongitudeSign = LongitudeSign
    msg.LatitudeSign = LatitudeSign
    #wywolanie funkcji przeslania wiadomosci
    publish(msg)

#Parse set servos frame
def parseSetServos(frame):
    try:
        sail = int(frame[0], 16)
        rudder = int(frame[1], 16)
    except ValueError:
        print('Invalid parameter')
        return 0

    print('Set sail to {}, rudder to {}'.format(sail, rudder))
    return 1

def translateInputFrame(unpackFrame):
    i = 0
    unpackFrame = list(unpackFrame)
    frame = unpackFrame
    for element in unpackFrame:
        frame[i] = '{:02X}'.format(element)
        i+=1
    return frame

def translateOutputFrame(frame):
    i = 0
    for element in frame:
        frame[i] = int(('0x' + element),16)
        i+=1
    return frame

def reqBasicSensor():
  # Rzadanie danych
  frame = ['E2', '60', '00', 'F5', 'F5', 'E3']
  frame = translateOutputFrame(frame)

  return frame

#Dodanie znakow ucieczki
def setEscapeFlag(frame):
    i = 0
    while i < len(frame):
      if isFlag(frame[i]):
              new_frame = []
              start = frame[:i]
              sign = 'F5'
              end = frame[i:]
              new_frame.extend(start)
              new_frame.extend([sign])
              new_frame.extend(end)
              frame = new_frame
              i = i + 1
      i = i + 1
    return frame

def crcServoFrame(frame):
  # Obliczenie CRC dla otrzymanej ramki
  hash = crc8.crc8()
  for i in frame:
    hash.update(str(bytearray.fromhex(i)))

  calcCRC = (hash.hexdigest()).upper()

  return calcCRC

def setServoFrameFlags(frame,crc8l):
  new_frame = []
  extended_frame = setEscapeFlag(frame)
  new_frame.extend([START_FLAG])
  new_frame.extend(extended_frame)
  new_frame.extend([crc8l])
  new_frame.extend([END_FLAG])
  return new_frame

# Ustawnienie serwo
def setServoFrame(sail,rudder):
  if sail >=100: #Ograniczenie
      sail = 100
  if rudder >=100: #Ograniczenie
      rudder = 100
  hex_sail = '{:02X}'.format(sail)
  hex_rudder = '{:02X}'.format(rudder)
  #print(hex_sail ,hex_sail)
  frame = [FRAMETYPE_SET_SERVOS, '02', hex_sail, hex_rudder]
  crc8l = crcServoFrame(frame)
  frame = setServoFrameFlags(frame,crc8l)

  #postac ramki: frame = [START_FLAG, FRAMETYPE_SET_SERVOS, '02', hex_sail, hex_rudder, crc8,END_FLAG]
  return translateOutputFrame(frame)

def publish(msg):
    publisher = rospy.Publisher('realData', ShipData, queue_size=10)
    publisher.publish(msg)