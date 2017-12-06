'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity, array
from numpy.linalg import norm, inv
import numpy as np
from math import cos, sin, atan2, pi, asin, sqrt, acos, asin


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        mFoot2Hip = transform - [[0,0,0,0],[0,0,0,50],[0,0,0,-85],[0,0,0,0]]# -50 for RLeg
        vFoot2Hip = mFoot2Hip[0:3, 3]
        lUpperLeg = 100
        lLowerLeg = 102.9
        lHipAnkle = norm(vFoot2Hip)
        KneePitch = pi - np.arccos((lUpperLeg*lUpperLeg + lLowerLeg*lLowerLeg - lHipAnkle*lHipAnkle) / (2*lUpperLeg*lLowerLeg))
        print 'KneePitch: ' + str(KneePitch)
        AnklePitch1 = np.arccos((lLowerLeg*lLowerLeg + lHipAnkle*lHipAnkle - lUpperLeg*lUpperLeg) / (2*lLowerLeg*lHipAnkle))
        c = cos(pi/4)
        s = sin(pi/4)
        vFootToHipOrthogonal = np.dot([[1,0,0],[0,c,-s],[0,s,c]], vFoot2Hip)
        AnklePitch2 = atan2(vFootToHipOrthogonal[0], norm(vFootToHipOrthogonal[1:3]))
        AnklePitch = AnklePitch1 + AnklePitch2
        print 'AnklePitch: ' + str(AnklePitch)
        AnkleRoll = atan2(vFootToHipOrthogonal[1], vFootToHipOrthogonal[2])
        print 'AnkleRoll: ' + str(AnkleRoll)

        mFoot2Thigh = self.local_trans('LKneePitch', KneePitch)\
            .dot(self.local_trans('LAnklePitch', AnklePitch))\
            .dot(self.local_trans('LAnkleRoll', AnkleRoll))

        mHip2HipOrth = identity(4)
        mHip2HipOrth[0:3,0:3] = self.calculate_rotation_matrix(pi / 4, [1, 0, 0])

        mHip2Thigh = mFoot2Thigh\
            .dot(inv(mFoot2Hip))

        HipYawPitch = atan2(mHip2Thigh[1,0] * sqrt(2), mHip2Thigh[1,2])

        M = mHip2Thigh[0:3,0:3].dot(inv(self.calculate_rotation_matrix(HipYawPitch, [0,1,1])))

        HipPitch = atan2(-M[2,0], M[0,0]) -pi
        HipRoll = atan2(-M[1,2], M[1,1])

        self.print_matrix(mHip2Thigh, 'Hip2Thigh')
        self.print_matrix(inv(mHip2Thigh), 'THigh2Hip')

        print 'HipYawPitch: ' + str(HipYawPitch)
        print 'HipPitch: ' + str(HipPitch)
        print 'HipRoll: ' + str(HipRoll)


        joint_angles = [HipYawPitch, HipRoll, HipPitch, KneePitch, AnklePitch, AnkleRoll]

        return joint_angles

    def rotationMatrixToEulerAngles(self, R):
        sy = sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        if not sy < 1e-6:
            x = atan2(R[2, 1], R[2, 2])
            y = atan2(-R[2, 0], sy)
            z = atan2(R[1, 0], R[0, 0])
        else:
            x = atan2(-R[1, 2], R[1, 1])
            y = atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def print_matrix(self, matrix, name):
        angles = self.rotationMatrixToEulerAngles(matrix[0:3,0:3])
        print '-- ' + name + ': ' + str(np.round(angles, 2)) + ' --'
        print np.round(matrix, 2)
        print '----------------------------------'

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        angles = self.inverse_kinematics(effector_name, transform)

        #names.append("HeadYaw")
        #times.append([1.00000, 2.00000, 2.80000, 3.60000, 4.30000, 4.80000, 5.60000, 6.50000, 7.40000, 8.00000, 8.60000,
        #              10.50000])
        #keys.append([[0.02459, [3, -0.33333, 0.00000], [3, 0.33333, 0.00000]],
        #             [0.00000, [3, -0.33333, 0.01222], [3, 0.26667, -0.00977]],
        #             [-0.04138, [3, -0.26667, 0.00000], [3, 0.26667, 0.00000]],
        #             [-0.00000, [3, -0.26667, -0.00000], [3, 0.23333, 0.00000]],
        #             [-0.00000, [3, -0.23333, -0.00000], [3, 0.16667, 0.00000]],
        #             [-0.00000, [3, -0.16667, -0.00000], [3, 0.26667, 0.00000]],
        #             [0.51393, [3, -0.26667, 0.00000], [3, 0.30000, 0.00000]],
        #             [0.30224, [3, -0.30000, 0.12958], [3, 0.30000, -0.12958]],
        #             [-0.26354, [3, -0.30000, 0.00000], [3, 0.20000, 0.00000]],
        #             [-0.12043, [3, -0.20000, -0.04549], [3, 0.20000, 0.04549]],
        #             [0.00940, [3, -0.20000, 0.00000], [3, 0.63333, 0.00000]],
        #             [-0.06592, [3, -0.63333, 0.00000], [3, 0.00000, 0.00000]]])

        self.keyframes = ([], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
