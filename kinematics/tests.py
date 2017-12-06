
import numpy as np
from math import pi, sqrt, atan2, cos, sin
import random
import unittest
from forward_kinematics import ForwardKinematicsAgent
from inverse_kinematics import InverseKinematicsAgent


class Tests(unittest.TestCase):

    def all_chains(self, angles, expected_values):
        agent = ForwardKinematicsAgent()
        agent.forward_kinematics(angles)
        for key in expected_values:
            M = agent.transforms[key]

            #test rotation
            RotM = M[0:3,0:3]
            c = cos(expected_values[key][3])
            s = sin(expected_values[key][3])
            R = np.matrix([[1, 0, 0], [0, c, -s], [0, s, c]])
            c = cos(expected_values[key][4])
            s = sin(expected_values[key][4])
            P = np.matrix([[c, 0, s], [0, 1, 0], [-s, 0, c]])
            c = cos(expected_values[key][5])
            s = sin(expected_values[key][5])
            Y = np.matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            RotExp = np.dot(Y, np.dot(P, R))
            self.assertTrue(np.allclose(RotExp, RotM),
                            key + ': expected not equals calculated\n' + str(np.round(RotExp, 2)) + '\n' + str(np.round(RotM, 2)) + '\n' + str(self.rotationMatrixToEulerAngles(RotM) / (2*pi)))

            #test position
            values = [M[0,3], M[1,3], M[2,3]]
            self.assertTrue(np.allclose(expected_values[key][0:3], values),
                            key + ': \n' + str(np.round(expected_values[key][0:3], 2)) + ' expected not equals calculated \n' + str(np.round(values, 2)))

    def single_matrix(self, agent, joint_name, angle_in_radians, result_matrix):
        self.assertTrue(np.allclose(agent.local_trans(joint_name, angle_in_radians), result_matrix),
                        str(agent.local_trans(joint_name, angle_in_radians)) + ' calculated not equals expected ' + str(result_matrix))

    def test_local_trans(self):
        agent = ForwardKinematicsAgent()

        self.single_matrix(agent, 'HeadYaw', 0, [[1, 0, 0, 0],
                                                 [0, 1, 0, 0],
                                                 [0, 0, 1, 126.5],
                                                 [0, 0, 0, 1]])
        self.single_matrix(agent, 'HeadYaw', pi/2, [[0, -1, 0, 0],
                                                   [1, 0, 0, 0],
                                                   [0, 0, 1, 126.5],
                                                   [0, 0, 0, 1]])
        self.single_matrix(agent, 'HeadPitch', -pi/2, [[0, 0, -1, 0],
                                                       [0, 1, 0, 0],
                                                       [1, 0, 0, 0],
                                                       [0, 0, 0, 1]])
        #self.single_matrix(agent, 'RElbowRoll', pi, [[1, 0, 0, 0],
        #                                             [0, -1, 0, 0],
        #                                             [0, 0, -1, 0],
        #                                             [0, 0, 0, 1]])

    def test_rotationMatrixToEulerAngles(self):
        c = cos(pi/4)
        s = sin(pi/4)
        R = np.matrix([[1,0,0],[0,c,-s],[0,s,c]])
        P = np.matrix([[c,0,s],[0,1,0],[-s,0,c]])
        Y = np.matrix([[c,-s,0],[s,c,0],[0,0,1]])
        angles = self.rotationMatrixToEulerAngles(R)
        self.assertTrue(np.allclose(np.array([pi/4, 0, 0]), angles),
                        str(angles) + ' not equals pi/4, 0, 0: ' + str(R))
        angles = self.rotationMatrixToEulerAngles(P)
        self.assertTrue(np.allclose(np.array([0, pi/4, 0]), angles),
                        str(angles) + ' not equals 0, pi/4, 0: ' + str(P))
        angles = self.rotationMatrixToEulerAngles(Y)
        self.assertTrue(np.allclose(np.array([0, 0, pi/4]), angles),
                        str(angles) + ' not equals 0, 0, pi/4: ' + str(Y))

        angles = np.array([pi / 3, 3 * pi / 4, pi / 4])
        c = cos(angles[0])
        s = sin(angles[0])
        R = np.matrix([[1,0,0],[0,c,-s],[0,s,c]])
        c = cos(angles[1])
        s = sin(angles[1])
        P = np.matrix([[c,0,s],[0,1,0],[-s,0,c]])
        c = cos(angles[2])
        s = sin(angles[2])
        Y = np.matrix([[c,-s,0],[s,c,0],[0,0,1]])
        M_1 = np.dot(Y, np.dot(P, R))

        angles = self.rotationMatrixToEulerAngles(M_1)
        c = cos(angles[0])
        s = sin(angles[0])
        R = np.matrix([[1,0,0],[0,c,-s],[0,s,c]])
        c = cos(angles[1])
        s = sin(angles[1])
        P = np.matrix([[c,0,s],[0,1,0],[-s,0,c]])
        c = cos(angles[2])
        s = sin(angles[2])
        Y = np.matrix([[c,-s,0],[s,c,0],[0,0,1]])
        M_2 = np.dot(Y, np.dot(P, R))
        self.assertTrue(np.allclose(M_1, M_2),
                        '\n' + str(M_1) + 'not equals\n' + str(M_2))
        pass

    def test_default_values(self):
        angles = {'HeadYaw' : 0,
                  'HeadPitch' : 0,
                  'LShoulderPitch' : 0,
                  'LShoulderRoll' : 0,
                  'LElbowYaw' : 0,
                  'LElbowRoll' : 0,
                  'LWristYaw' : 0,
                  'LHipYawPitch' : 0,
                  'LHipRoll' : 0,
                  'LHipPitch' : 0,
                  'LKneePitch' : 0,
                  'LAnklePitch' : 0,
                  'LAnkleRoll' : 0,
                  'RShoulderPitch' : 0,
                  'RShoulderRoll' : 0,
                  'RElbowYaw' : 0,
                  'RElbowRoll' : 0,
                  'RWristYaw' : 0,
                  'RHipYawPitch' : 0,
                  'RHipRoll' : 0,
                  'RHipPitch' : 0,
                  'RKneePitch' : 0,
                  'RAnklePitch' : 0,
                  'RAnkleRoll' : 0}
        expected_values = {'HeadPitch' : [0, 0, 126.5, 0, 0, 0],
                           'LWristYaw':  [160.95, 113, 100, 0, 0, 0],
                           'RWristYaw':  [160.95, -113, 100, 0, 0, 0],
                           'LAnkleRoll': [0, 50, -287.9, 0, 0, 0],
                           'RAnkleRoll': [0, -50, -287.9, 0, 0, 0]
                           }
        self.all_chains(angles, expected_values)
        pass

    def test_head_values(self):
        angles = {'HeadYaw' : pi/2,
                  'HeadPitch' : pi/4,
                  'LShoulderPitch' : 0,
                  'LShoulderRoll' : 0,
                  'LElbowYaw' : 0,
                  'LElbowRoll' : 0,
                  'LWristYaw' : 0,
                  'LHipYawPitch' : 0,
                  'LHipRoll' : 0,
                  'LHipPitch' : 0,
                  'LKneePitch' : 0,
                  'LAnklePitch' : 0,
                  'LAnkleRoll' : 0,
                  'RShoulderPitch' : 0,
                  'RShoulderRoll' : 0,
                  'RElbowYaw' : 0,
                  'RElbowRoll' : 0,
                  'RWristYaw' : 0,
                  'RHipYawPitch' : 0,
                  'RHipRoll' : 0,
                  'RHipPitch' : 0,
                  'RKneePitch' : 0,
                  'RAnklePitch' : 0,
                  'RAnkleRoll' : 0}
        expected_values = {'HeadYaw':  [0, 0, 126.5, 0, 0, pi/2],
                           'HeadPitch' : [0, 0, 126.5, 0, pi/4, pi/2]
                           }
        self.all_chains(angles, expected_values)
        pass

    def test_arm_values(self):
        angles = {'HeadYaw' : 0,
                  'HeadPitch' : 0,
                  'LShoulderPitch' : pi/2,
                  'LShoulderRoll' : pi/4,
                  'LElbowYaw' : -pi/2,
                  'LElbowRoll' : -pi/2,
                  'LWristYaw' : 0,
                  'LHipYawPitch' : 0,
                  'LHipRoll' : 0,
                  'LHipPitch' : 0,
                  'LKneePitch' : 0,
                  'LAnklePitch' : 0,
                  'LAnkleRoll' : 0,
                  'RShoulderPitch' : pi/2,
                  'RShoulderRoll' : -pi/4,
                  'RElbowYaw' : pi/2,
                  'RElbowRoll' : pi/2,
                  'RWristYaw' : 0,
                  'RHipYawPitch' : 0,
                  'RHipRoll' : 0,
                  'RHipPitch' : 0,
                  'RKneePitch' : 0,
                  'RAnklePitch' : 0,
                  'RAnkleRoll' : 0}
        expected_values = {'RShoulderPitch': [0, -98, 100, 0, pi/2, 0],
                           'RShoulderRoll': [0, -98, 100, -pi/2, pi/4, -pi/2],
                           'RElbowYaw': [0, -98 - 15 * cos(-pi / 4) + 105 * sin(-pi / 4), 100 - 15 * sin(-pi / 4) - 105 * cos(-pi / 4), 0, pi/4, -pi/2],
                           'RWristYaw': [55.95, -98 - 15 * cos(-pi / 4) + 105 * sin(-pi / 4), 100 - 15 * sin(-pi / 4) - 105 * cos(-pi / 4), pi/4, 0, 0],
                           'LShoulderPitch': [0, 98, 100, 0, pi / 2, 0],
                           'LShoulderRoll': [0, 98, 100, pi / 2, pi / 4, pi / 2],
                           'LElbowYaw': [0, 98 + 15 * cos(-pi / 4) - 105 * sin(-pi / 4), 100 - 15 * sin(-pi / 4) - 105 * cos(-pi / 4), 0, pi / 4, pi / 2],
                           'LWristYaw': [55.95, 98 + 15 * cos(-pi / 4) - 105 * sin(-pi / 4), 100 - 15 * sin(-pi / 4) - 105 * cos(-pi / 4), -pi/4, 0, 0]
                           }
        self.all_chains(angles, expected_values)
        pass

    def test_leg_values(self):
        angles = {'HeadYaw' : 0,
                  'HeadPitch' : 0,
                  'LShoulderPitch' : 0,
                  'LShoulderRoll' : 0,
                  'LElbowYaw' : 0,
                  'LElbowRoll' : 0,
                  'LWristYaw' : 0,
                  'LHipYawPitch' : -pi/2,
                  'LHipRoll' : pi/4,
                  'LHipPitch' : -pi/4,
                  'LKneePitch' : pi/2,
                  'LAnklePitch' : pi/4,
                  'LAnkleRoll' : pi/2,
                  'RShoulderPitch' : 0,
                  'RShoulderRoll' : 0,
                  'RElbowYaw' : 0,
                  'RElbowRoll' : 0,
                  'RWristYaw' : 0,
                  'RHipYawPitch' : -pi/2,
                  'RHipRoll' : -pi/4,
                  'RHipPitch' : -pi/4,
                  'RKneePitch' : pi/2,
                  'RAnklePitch' : pi/4,
                  'RAnkleRoll' : -pi/2,}
        expected_values = {'LHipYawPitch': [0, 50, -85, -pi/4, -pi/4, pi/2],
                           'LHipRoll': [0, 50, -85, 0, -pi/4, pi/2],
                           'LHipPitch': [0, 50, -85, 0, -pi/2, pi/2],
                           'LKneePitch': [0, 150, -85, 0, 0, pi/2],
                           'LAnklePitch': [0, 150, -85-102.9, 0, pi/4, pi/2],
                           'LAnkleRoll': [0, 150, -85-102.9, pi/2, pi/4, pi/2],
                           'RHipYawPitch': [0, -50, -85, pi / 4, -pi / 4, -pi / 2],
                           'RHipRoll': [0, -50, -85, 0, -pi / 4, -pi / 2],
                           'RHipPitch': [0, -50, -85, 0, -pi / 2, -pi / 2],
                           'RKneePitch': [0, -150, -85, 0, 0, -pi / 2],
                           'RAnklePitch': [0, -150, -85 - 102.9, 0, pi / 4, -pi / 2],
                           'RAnkleRoll': [0, -150, -85 - 102.9, -pi / 2, pi / 4, -pi / 2],
                           }
        self.all_chains(angles, expected_values)
        pass

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



    def test_inverse_kinematics(self):

        agent = InverseKinematicsAgent()

        hip = float(random.randrange(-65, 42))/360*pi*2
        values = {
            'HeadYaw': float(random.randrange(-119, 119)),
            'HeadPitch': float(random.randrange(-38, 29)),

            'LShoulderPitch': float(random.randrange(-119, 119)),
            'LShoulderRoll': float(random.randrange(-18, 76)),
            'LElbowYaw': float(random.randrange(-119, 119)),
            'LElbowRoll': float(random.randrange(-88, -2)),
            'LWristYaw': float(random.randrange(-104, 104)),

            'RShoulderPitch': float(random.randrange(-119, 119)),
            'RShoulderRoll': float(random.randrange(-76, 18)),
            'RElbowYaw': float(random.randrange(-119, 119)),
            'RElbowRoll': float(random.randrange(2, 88)),
            'RWristYaw': float(random.randrange(-104, 104)),

            'LHipYawPitch': -pi / 2,
            'LHipRoll': pi / 4,
            'LHipPitch': -pi / 4,
            'LKneePitch': pi / 2,
            'LAnklePitch': pi / 4,
            'LAnkleRoll': pi / 2,

            'RHipYawPitch': hip,
            'RHipRoll': float(random.randrange(-45, 21)),
            'RHipPitch': float(random.randrange(-88, 27)),
            'RKneePitch': float(random.randrange(-5, 121)),
            'RAnklePitch': float(random.randrange(-68, 52)),
            'RAnkleRoll': float(random.randrange(-44, 22))
        }

        agent.forward_kinematics(values)
        result = agent.inverse_kinematics('LLeg', agent.transforms['LAnkleRoll'])

        self.assertTrue(np.isclose(values['LHipYawPitch'], result[0], 0.05)     , 'LHipYawPitch failed, expected angle:\n' + str(values['LHipYawPitch']) + 'result angle:\n' + str(result[0]))
        self.assertTrue(np.isclose(values['LHipRoll'], result[1], 0.05)         , 'LHipRoll failed, expected angle:\n' + str(values['LHipRoll']) + 'result angle:\n' + str(result[1]))
        self.assertTrue(np.isclose(values['LHipPitch'], result[2], 0.05)        , 'LHipPitch failed, expected angle:\n' + str(values['LHipPitch']) + 'result angle:\n' + str(result[2]))
        self.assertTrue(np.isclose(values['LKneePitch'], result[3], 0.05)       , 'LKneePitch failed, expected angle:\n' + str(values['LKneePitch']) + 'result angle:\n' + str(result[3]))
        self.assertTrue(np.isclose(values['LAnklePitch'], result[4], 0.05)      , 'LAnklePitch failed, expected angle:\n' + str(values['LAnklePitch']) + 'result angle:\n' + str(result[4]))
        self.assertTrue(np.isclose(values['LAnkleRoll'], result[5], 0.05)       , 'LAnkleRoll failed, expected angle:\n' + str(values['LAnkleRoll']) + 'result angle:\n' + str(result[5]))

#        self.assertTrue(values['RHipYawPitch'] == result['RLeg'][0]     , 'RHipYawPitch failed, expected angle:\n' + str(values['RHipYawPitch']) + 'result angle:\n' + str(result['RLeg'][0]))
#        self.assertTrue(values['RHipRoll'] == result['RLeg'][1]         , 'RHipRoll failed, expected angle:\n' + str(values['RHipRoll']) + 'result angle:\n' + str(result['RLeg'][1]))
#        self.assertTrue(values['RHipPitch'] == result['RLeg'][2]        , 'RHipPitch failed, expected angle:\n' + str(values['RHipPitch']) + 'result angle:\n' + str(result['RLeg'][2]))
#        self.assertTrue(values['RKneePitch'] == result['RLeg'][3]       , 'RKneePitch failed, expected angle:\n' + str(values['RKneePitch']) + 'result angle:\n' + str(result['RLeg'][3]))
#        self.assertTrue(values['RAnklePitch'] == result['RLeg'][4]      , 'RAnklePitch failed, expected angle:\n' + str(values['RAnklePitch']) + 'result angle:\n' + str(result['RLeg'][4]))
#        self.assertTrue(values['RAnkleRoll'] == result['RLeg'][5]       , 'RAnkleRoll failed, expected angle:\n' + str(values['RAnkleRoll']) + 'result angle:\n' + str(result['RLeg'][5]))

if __name__ == '__main__':
    unittest.main()

