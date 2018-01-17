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
from numpy.matlib import identity
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
        AnklePitch1 = np.arccos((lLowerLeg*lLowerLeg + lHipAnkle*lHipAnkle - lUpperLeg*lUpperLeg) / (2*lLowerLeg*lHipAnkle))
        c = cos(pi/4)
        s = sin(pi/4)
        vFootToHipOrthogonal = np.dot([[1,0,0],[0,c,-s],[0,s,c]], vFoot2Hip)
        AnklePitch2 = atan2(vFootToHipOrthogonal[0], norm(vFootToHipOrthogonal[1:3]))
        AnklePitch = AnklePitch1 + AnklePitch2
        AnkleRoll = atan2(vFootToHipOrthogonal[1], vFootToHipOrthogonal[2])

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

        joint_angles = [HipYawPitch, HipRoll, HipPitch, KneePitch, AnklePitch, AnkleRoll]

        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        if (effector_name != "LLeg"):
            return

        angles = self.inverse_kinematics(effector_name, transform)

        names = list()
        times = list()
        keys = list()
        for chain_name in self.chains:
            if (chain_name == 'LLeg'):
                i= 0
                for joint_name in self.chains['LLeg']:
                    names.append(joint_name)
                    times.append([2.0])
                    keys.append([[angles[i], [3, 0.00000, 0.00000], [3, 0.00000, 0.00000]]])
                    i = i+1
            else:
                for joint_name in self.chains[chain_name]:
                    names.append(joint_name)
                    times.append([2.0])
                    keys.append([[0, [3, 0.00000, 0.00000], [3, 0.00000, 0.00000]]])



        self.keyframes = (names, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
