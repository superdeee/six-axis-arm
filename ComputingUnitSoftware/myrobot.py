import numpy as np
import math
#from mpl_toolkits import mplot3d
#import matplotlib.pyplot as plt
import serial
import time

class MyRobot:
    CLK_F = 90000000
    TIM_PRESCALER = 10
    TIM_F = CLK_F / TIM_PRESCALER
    TIM_T = 1 / TIM_F

    pos = [0, 0, 0, 0, 0, 0]
    end = 0
    steps_per_rev_joints = [32000, 320000, 168000, 25600, 20000, 6400]
    v_joints = [0.09375, 0.05625 / 2, 0.05625 / 2, 0.5625, 0.9, 2.8125]  # obr/s
    # a_joints = [0.50525, 4.1278, 4.1278, 4.1278, 4.1278, 4.1278]  # obr/s^2
    # a_joints = [0.50525, 0.413265306122449, 0.7871720116618076, 5.165816326530613, 6.612244897959184, 20.66326530612245]  # obr/s^2
    deleje = [159100, 50311, 69436, 177878, 201246, 355756]
    a_joints = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

    def __init__(self, comPort, baudRate):
        if self.establishSerialConnection(comPort, baudRate):
            self.USE_SERIAL = True
            print('Serial communication started on ' + comPort)
        else:
            self.USE_SERIAL = False
            print('Serial communication failed')

    def establishSerialConnection(self, comPort, baud):
        try:
            self.ser = serial.Serial(
                port=comPort,
                baudrate=baud,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            return True

        except:
            return False

    @staticmethod
    def DHparams(a=0, b=0, c=0, d=0, e=0, f=0):

        return [[a, 132, 0, np.pi / 2],
                [b, 0, 148.75, 0],
                [c, 0, 0, np.pi / 2],
                [d, 160, 0, -np.pi / 2],
                [e, 0, 0, np.pi / 2],  # +np.pi linijke nizej -37
                # [f,     100,        0,  -np.pi/2]]
                [f, 100, 0, 0]]

    @staticmethod
    def DH(theta, wspS, wspA, alfa):
        sinA = np.sin(alfa)
        cosA = np.cos(alfa)
        sinT = np.sin(theta)
        cosT = np.cos(theta)

        macierzTrans = [[cosT, -1 * sinT * cosA, 1 * sinT * sinA, wspA * cosT],
                        [sinT, cosT * cosA, -1 * cosT * sinA, wspA * sinT],
                        [0, sinA, cosA, wspS],
                        [0, 0, 0, 1]]

        return np.array(macierzTrans)

    @staticmethod
    def getGoalMatrix(x, y, z, r, p, yw): #katy w raianach

        goalArr = np.eye(4)

        goalArr[0, 0] = np.cos(yw) * np.cos(p)
        goalArr[0, 1] = -np.sin(yw) * np.cos(r) + np.cos(yw) * np.sin(p) * np.sin(r)
        goalArr[0, 2] = np.sin(yw) * np.sin(r) + np.cos(yw) * np.sin(p) * np.cos(r)

        goalArr[1, 0] = np.sin(yw) * np.cos(p)
        goalArr[1, 1] = np.cos(yw) * np.cos(r) + np.sin(yw) * np.sin(p) * np.sin(r)
        goalArr[1, 2] = -np.cos(yw) * np.sin(r) + np.sin(yw) * np.sin(p) * np.cos(r)

        goalArr[2, 0] = -np.sin(p)
        goalArr[2, 1] = np.cos(p) * np.sin(r)
        goalArr[2, 2] = np.cos(p) * np.cos(r)

        goalArr[0, 3] = x
        goalArr[1, 3] = y
        goalArr[2, 3] = z

        return goalArr

    @staticmethod
    def fwdKine(a, b, c, d, e, f, plot=False):

        kinMatrix = MyRobot.DHparams(a, b, c, d, e, f)

        R01 = np.dot(np.eye(4), MyRobot.DH(*kinMatrix[0]))
        R02 = np.dot(R01, MyRobot.DH(*kinMatrix[1]))
        R03 = np.dot(R02, MyRobot.DH(*kinMatrix[2]))
        R04 = np.dot(R03, MyRobot.DH(*kinMatrix[3]))
        R05 = np.dot(R04, MyRobot.DH(*kinMatrix[4]))
        R06 = np.dot(R05, MyRobot.DH(*kinMatrix[5]))

        if plot:
            x = [0]
            y = [0]
            z = [0]

            x.append(R01[0, 3])
            y.append(R01[1, 3])
            z.append(R01[2, 3])

            x.append(R02[0, 3])
            y.append(R02[1, 3])
            z.append(R02[2, 3])

            x.append(R03[0, 3])
            y.append(R03[1, 3])
            z.append(R03[2, 3])

            x.append(R04[0, 3])
            y.append(R04[1, 3])
            z.append(R04[2, 3])

            x.append(R05[0, 3])
            y.append(R05[1, 3])
            z.append(R05[2, 3])

            x.append(R06[0, 3])
            y.append(R06[1, 3])
            z.append(R06[2, 3])

            fig = plt.figure()
            ax = plt.axes(projection='3d')

            ax.plot(x, y, z)
            ax.scatter(x, y, z, c='r')
            ax.set_xlabel('X axis')
            ax.set_ylabel('Y axis')
            ax.set_zlabel('Z axis')

            ax.set_xlim3d(-200, 200)
            ax.set_ylim3d(-200, 200)
            ax.set_zlim3d(0, 400)

            # print('\nX: {} \nY: {} \nZ: {}'.format(x[-1], y[-1], z[-1]))

            plt.show()

        return R06

    @staticmethod
    def invKine(goal):

        param = MyRobot.DHparams()

        dL0 = param[0][1]
        dL1 = param[1][2]
        dL2 = param[3][1]
        dL4 = param[5][1]

        endpoint_orient = goal[0:3, 0:3]  # endpoint_orient[0] = x, endpoint_orient[1] = y, endpoint_orient[2] = z
        endpoint_pos = goal[0:3, 3]  # endpoint_pos[0] = x, endpoint_pos[1] = y, endpoint_pos[2] = z

        wrist_pos = endpoint_pos - endpoint_orient[0:3, 2] * dL4
        fi1 = math.atan2(wrist_pos[1], wrist_pos[0])

        try:
            wekXY = np.sqrt(wrist_pos[0] ** 2 + wrist_pos[1] ** 2)
            fi_B_goal = math.atan2(wrist_pos[2] - dL0, wekXY)  # kat os2-goal
            DW = np.sqrt(wrist_pos[0] ** 2 + wrist_pos[1] ** 2 + (wrist_pos[2] - dL0) ** 2)
            fi_miedzy_osiami = math.acos((dL1 ** 2 + dL2 ** 2 - DW ** 2) / (2 * dL1 * dL2))

        except:
            raise Exception('OUT OF BOUNDS FOR J2')

        try:
            kat_osi_B = math.acos((dL1 ** 2 + DW ** 2 - dL2 ** 2) / (2 * dL1 * DW)) + fi_B_goal
        except:
            raise Exception('OUT OF BOUNDS FOR J3')

        a = fi1
        b = kat_osi_B
        c = fi_miedzy_osiami - np.pi / 2
        d = d2 = e =e2 = f = f2 = 0

        czlony = MyRobot.DHparams(a, b, c, d, e, f)

        R01 = np.dot(np.eye(4), MyRobot.DH(*czlony[0]))
        R02 = np.dot(R01, MyRobot.DH(*czlony[1]))
        R03 = np.dot(R02, MyRobot.DH(*czlony[2]))

        R36 = np.dot(R03[0:3, 0:3].T, endpoint_orient)

        try:
            d = math.atan2(R36[1, 2], R36[0, 2])
            d2 = math.atan2(-R36[1, 2], -R36[0, 2])
            e = math.atan2(np.sqrt(R36[0, 2] ** 2 + R36[1, 2] ** 2), R36[2, 2])
            e2 = math.atan2(-np.sqrt(R36[0, 2] ** 2 + R36[1, 2] ** 2), R36[2, 2])
            f = math.atan2(R36[2, 1], -R36[2, 0])
            f2 = math.atan2(-R36[2, 1], R36[2, 0])
        except:
            raise Exception('CANT SOLVE WRIST')

        return a, b, c, d, e, f, d2, e2, f2

    def stepPos(self, a=pos[0], b=pos[1], c=pos[2], d=pos[3], e=pos[4], f=pos[5]):

        a_robot = a / 2 / np.pi * self.steps_per_rev_joints[0]
        b_robot = (np.pi / 2 - b) / 2 / np.pi * self.steps_per_rev_joints[1]
        c_robot = (np.pi / 2 - c) / 2 / np.pi * self.steps_per_rev_joints[2]
        d_robot = -d / 2 / np.pi * self.steps_per_rev_joints[3]
        e_robot = -e / 2 / np.pi * self.steps_per_rev_joints[4]
        f_robot = -f / 2 / np.pi * self.steps_per_rev_joints[5]

        return np.round([a_robot, b_robot, c_robot, d_robot, e_robot, f_robot]).astype(int)

    @staticmethod
    def approximateMove(steps, steps_per_rev, v_max, a):

        v_calc = np.sqrt((steps / steps_per_rev) * a)
        if steps != 0:
            t_calc = min(v_max, v_calc) / a + steps / steps_per_rev / min(v_max, v_calc)
        else:
            t_calc = 0

        return t_calc, min(v_max, v_calc)

    @staticmethod
    def fitVelocity(t, steps, steps_per_rev, a):

        if steps == 0: return 0
        delta = max(t ** 2 - 4 * steps / steps_per_rev / a, 0)
        v1 = (t - np.sqrt(delta)) / (2 / a)
        v2 = (t + np.sqrt(delta)) / (2 / a)
        #print(v1,v2)
        return min(v1, v2)

    @staticmethod
    def jMove(move, stepsRev, v_base, a_base):
        t_base = []
        t_new = []
        v_new = []

        for nr, joint in enumerate(move, 0):
            t_joint, v_joint = MyRobot.approximateMove(joint, stepsRev[nr], v_base[nr], a_base[nr])
            t_base.append(t_joint)

        for nr, joint in enumerate(move, 0):
            v_fit = MyRobot.fitVelocity(max(t_base), joint, stepsRev[nr], a_base[nr])
            v_new.append(v_fit)

        for nr, joint in enumerate(move, 0):
            t_joint, v_joint = MyRobot.approximateMove(joint, stepsRev[nr], v_new[nr], a_base[nr])
            t_new.append(t_joint)

        v_new = np.array(v_new)
        v_new[v_new == 0] = 0.01
        spd = np.round(MyRobot.TIM_F / v_new / stepsRev, decimals=0)
        spd = [65535 if x > 65535 else x for x in spd]

        #print('move: ', move)
        #print('spd: ', spd)
        #print('v_base: ', v_base)
        #print('v_new: ', v_new)
        #print('t_base: ', t_base)
        #print('t_new: ', t_new)

        return spd, np.max(t_new)

    def serialSendMove(self, new_pos, spd):
        frm = ''

        axis_letter = ['A', 'B', 'C', 'D', 'E', 'F']

        for nr in range(6):
            if new_pos[nr] != self.pos[nr]:
                frm += '{l}{m:X<7}V{s:X<5}'.format(l=axis_letter[nr], m=int(new_pos[nr]), s=int(spd[nr]))
        if self.USE_SERIAL:
            self.ser.write(str.encode(frm))
            #print(str.encode(frm))
        else:
            print(str.encode(frm))

    def serialSendEffector(self, chwytak_pos):

        frm = 'H{m:X<7}V00000'.format(m=int(chwytak_pos))

        if self.USE_SERIAL:
            self.ser.write(str.encode(frm))
        else:
            print(str.encode(frm))

    def setPosJoint(self, x, y, z, r, p, yw):

        goal = self.getGoalMatrix(x, y, z, r, p, yw)

        # try:
        a, b, c, d, e, f, d2, e2, f2 = self.invKine(goal)
        # except:
        #     print('CANT FIND SOLUTION - MOVE CANCELED')
        #     return


        MyRobot.fwdKine(a, b, c, d2, e2, f2, False)
        new_pos = self.stepPos(a, b, c, d2, e2, f2)
        move = np.abs(new_pos - self.pos)
        speeds, tMax = self.jMove(move, self.steps_per_rev_joints, self.v_joints, self.a_joints)
        self.serialSendMove(new_pos, speeds)

        self.pos = new_pos
        return tMax

    def setEnd(self, end_pos):

        self.serialSendEffector(end_pos)
        self.end = end_pos

if __name__ == "__main__":

   # robot = MyRobot('COM6', 115200)
    robot = MyRobot('/dev/ttyAMA0', 115200)

    time.sleep(1.2 * robot.setPosJoint(200, 0, 100, np.pi, 0, 0))
    time.sleep(1.2 * robot.setPosJoint(87, 270, 100, np.pi, 0, 0))
    time.sleep(1.2 * robot.setPosJoint(87, 270, 45, np.pi, 0, 0))
   # time.sleep(1.2 * robot.setPosJoint(80, 270, 100, np.pi, 0, 0))
   # time.sleep(1.2 * robot.setPosJoint(200, 0, 100, np.pi, 0, 0))

    input()
    robot.serialSendMove([0, 0, 0, 0, 0, 0], [1000, 1000, 1000, 1000, 1000, 1000])

