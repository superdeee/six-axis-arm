import numpy as np
from kamera import klasyfikuj
from myrobot import MyRobot
#from matplotlib import pyplot
import time
import RPi.GPIO as GPIO

def przelozDo(P):

    time.sleep(1.1 * robot.setPosJoint(87, 270, 100, np.pi, 0, 0))
    time.sleep(1.1 * robot.setPosJoint(87, 270, 45, np.pi, 0, 0))

    robot.setEnd(13)
    time.sleep(0.7)

    time.sleep(1.1 * robot.setPosJoint(87, 270, 100, np.pi, 0, 0))

    time.sleep(1.1 * robot.setPosJoint(*P, np.pi, 0, 0))

    robot.setEnd(1)
    time.sleep(0.7)

a=0
boxOne = None
boxTwo = None
boxThree = None

# robot = MyRobot('COM6', 115200)
robot = MyRobot('/dev/ttyAMA0', 115200)
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.OUT)
GPIO.output(21, GPIO.LOW)

P1 = [200, -150, 100]
P2 = [200, 0, 100]
P3 = [200, 150, 100]

POdrzut = [-60, -200, 100]

tryb = 'ksztalt'

time.sleep(1.1* robot.setPosJoint(200, 0, 100, np.pi, 0, 0))
robot.setEnd(1)
input('start')
time.sleep(0.7)

while a<5:
    a+=1
    shpOK, shpName, shpPercent, colOK, colName, colPercent = klasyfikuj()

    if tryb == 'ksztalt':
        cecha = shpName
        cechaOK = shpOK
        print(shpOK, shpName, shpPercent)

    elif tryb == 'kolor':
        cecha = colName
        cechaOK = colOK
        print(colOK, colName, colPercent)

    if (boxOne == None or boxOne == cecha) and cechaOK:
        boxOne = cecha
        print('wlozono do 1')
        przelozDo(P1)
        time.sleep(1.1 * robot.setPosJoint(200, 0, 100, np.pi, 0, 0))
    elif (boxTwo == None or boxTwo == cecha) and cechaOK:
        boxTwo = cecha
        print('wlozono do 2')
        przelozDo(P2)
        time.sleep(1.1 * robot.setPosJoint(200, 0, 100, np.pi, 0, 0))
    elif (boxThree == None or boxThree == cecha) and cechaOK:
        boxThree = cecha
        print('wlozono do 3')
        przelozDo(P3)
        time.sleep(1.1 * robot.setPosJoint(200, 0, 100, np.pi, 0, 0))
    else:
        print('wlozono do odrzutu')
        GPIO.output(21, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(21, GPIO.LOW)
        przelozDo(POdrzut)
        time.sleep(1.1 * robot.setPosJoint(200, 0, 100, np.pi, 0, 0))

    input('Czy nastepny przedmiot załadowany ?')

print('1: ', boxOne)
print('2: ', boxTwo)
print('3: ', boxThree)

robot.serialSendMove([0, 0, 0, 0, 0, 0], [1000, 1000, 1000, 1000, 1000, 1000])
GPIO.cleanup()
