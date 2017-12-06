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
from math import cos, sin, atan2, pi, asin, sqrt


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

        #print self.local_trans('LKneePitch', KneePitch)
        #print self.local_trans('LAnklePitch', AnklePitch)
        #print self.local_trans('LAnkleRoll', AnkleRoll)
        mFoot2Thigh = mFoot2Hip\
            .dot(inv(self.local_trans('LAnkleRoll', AnkleRoll)))\
            .dot(inv(self.local_trans('LAnklePitch', AnklePitch)))\
            .dot(inv(self.local_trans('LKneePitch', KneePitch)))

        theMatrix = self.local_trans('LKneePitch', KneePitch)\
            .dot(self.local_trans('LAnklePitch', AnklePitch))\
            .dot(self.local_trans('LAnkleRoll', AnkleRoll))\
            .dot(inv(mFoot2Hip))[0:3,0:3]
        self.print_matrix(theMatrix, 'The Matrix')

        mHip2HipOrth = self.calculate_rotation_matrix(pi / 4, [1, 0, 0])
        mFoot2HipOrth = mHip2HipOrth.dot(mFoot2Hip[0:3,0:3])

        mHipOrthogonal2Foot = inv(mFoot2HipOrth)

        mHipOrthogonal2Thigh = mFoot2Thigh[0:3,0:3].dot(mHipOrthogonal2Foot)

        mHip2ThighReal = self.local_trans('LHipYawPitch', -pi/2)\
            .dot(self.local_trans('LHipRoll', pi/4))\
            .dot(self.local_trans('LHipPitch', -pi/4))
        mHip2Thigh = mHipOrthogonal2Thigh.dot(mHip2HipOrth)
        self.print_matrix(mHip2Thigh, 'Hip2Thigh')
        self.print_matrix(mHip2ThighReal[0:3,0:3], 'correct Hip2Thigh')

        #mHipOrthogonal2Thigh1 = mFoot2Thigh[0:3,0:3].dot(inv(rot))


        #print np.round(mFoot2Thigh, 1)
        #print self.rotationMatrixToEulerAngles(mFoot2Thigh[0:3,0:3])
        print self.rotationMatrixToEulerAngles(mHipOrthogonal2Thigh)
        HipYaw = atan2(-mHipOrthogonal2Thigh[0,1], mHipOrthogonal2Thigh[1,1])
        print 'HipYaw: ' + str(HipYaw)
        HipPitch = atan2(-mHipOrthogonal2Thigh[2,0], mHipOrthogonal2Thigh[2,2])
        print 'HipPitch: ' + str(HipPitch)
        HipRoll = asin(mHipOrthogonal2Thigh[2,1]) - pi/4
        print 'HipRoll: ' + str(HipRoll)
        joint_angles = []

        #mThigh2Foot = self.local_trans('LKneePitch', KneePitch)\
        #    .dot(self.local_trans('LAnklePitch', AnklePitch))\
        #    .dot(self.local_trans('LAnkleRoll', AnkleRoll))
        #rotation = identity(4)
        #rotation[0:3, 0:3] = self.calculate_rotation_matrix(pi/4, [1,0,0])
        #print rotation
        #mFoot2HipOrth = rotation.dot(mFoot2Hip)
        #print np.round(mFoot2HipOrth, 2)
        #mFoot2HipOrth = inv(mFoot2HipOrth)
        #print np.round(mFoot2HipOrth, 2)
        #mHipOrthogonal2Thigh = inv(mThigh2Foot).dot(mFoot2HipOrth)
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
        angles = self.rotationMatrixToEulerAngles(matrix)
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
