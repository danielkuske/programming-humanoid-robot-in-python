'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity, dot, transpose, array
from numpy.linalg import norm
from math import cos, sin
from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
                       }

        self.offsets = {'HeadYaw': [0, 0, 126.5],
                        'HeadPitch': [0, 0, 0],
                        'LShoulderPitch': [0, 98, 100],
                        'LShoulderRoll': [0, 0, 0],
                        'LElbowYaw': [105, 15, 0],
                        'LElbowRoll': [0, 0, 0],
                        'LWristYaw': [55.95, 0, 0],
                        'LHipYawPitch': [0, 50, -85],
                        'LHipRoll': [0, 0, 0],
                        'LHipPitch': [0, 0, 0],
                        'LKneePitch': [0, 0, -100],
                        'LAnklePitch': [0, 0, -102.9],
                        'LAnkleRoll': [0, 0, 0],
                        'RShoulderPitch': [0, -98, 100],
                        'RShoulderRoll': [0, 0, 0],
                        'RElbowYaw': [105, -15, 0],
                        'RElbowRoll': [0, 0, 0],
                        'RWristYaw': [55.95, 0, 0],
                        'RHipYawPitch': [0, -50, -85],
                        'RHipRoll': [0, 0, 0],
                        'RHipPitch': [0, 0, 0],
                        'RKneePitch': [0, 0, -100],
                        'RAnklePitch': [0, 0, -102.9],
                        'RAnkleRoll': [0, 0, 0],
                        }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        R = identity(3)
        if "LHipYawPitch" in joint_name:
            R = self.calculate_rotation_matrix(joint_angle, array([0,1,-1]))
        elif "RHipYawPitch" in joint_name:
            R = self.calculate_rotation_matrix(-joint_angle, array([0,-1,-1]))
        elif ("Yaw" in joint_name and not "Elbow" in joint_name) or "ShoulderRoll" in joint_name or "ElbowRoll" in joint_name:
            R = self.calculate_rotation_matrix(joint_angle, array([0,0,1]))
        elif "Roll" in joint_name or "Yaw" in joint_name:
            R = self.calculate_rotation_matrix(joint_angle, array([1,0,0]))
        elif "Pitch" in joint_name:
            R = self.calculate_rotation_matrix(joint_angle, array([0,1,0]))

        T = identity(4)
        T[0, 3] = self.offsets[joint_name][0]
        T[1, 3] = self.offsets[joint_name][1]
        T[2, 3] = self.offsets[joint_name][2]
        T[0:3, 0:3] = R
        return T

    def calculate_rotation_matrix(self, angle, u):
        u = u / norm(u)
        x = u[0]
        y = u[1]
        z = u[2]
        c = cos(angle)
        s = sin(angle)
        return matrix(
            [[x*x*(1-c)+c,   x*y*(1-c)-z*s, x*z*(1-c)+y*s],
             [y*x*(1-c)+z*s, y*y*(1-c)+c,   y*z*(1-c)-x*s],
             [z*x*(1-c)-y*s, z*y*(1-c)+x*s, z*z*(1-c)+c]])

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():       #for all chains
            T = identity(4)                             #compute the transformation matrix
            for joint in chain_joints:                  #via all joints in the chain,
                angle = joints[joint]                   #according to the given angle.
                Tl = self.local_trans(joint, angle)     #First compute joint matrix,
                T = dot(T, Tl)                          #then multiply it to the chained matrices
                self.transforms[joint] = T              #and save it

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
