'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
from SimpleXMLRPCServer import SimpleXMLRPCServer
import xmlrpclib
from numpy.matlib import identity
from agent_server import ServerAgent
from threading import Thread
import threading
import time
from keyframes import leftBackToStand

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''

    def __init__(self, obj):
        self.obj = obj
        self.proxy = weakref.proxy(obj)


    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        self.obj.server.execute_keyframes(keyframes)

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        self.obj.server.set_transform(effector_name, transform)

        #Thread(self.obj.server.set_transform(effector_name, transform)).start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''

    # YOUR CODE HERE

    def __init__(self):
        self.post = PostHandler(self)
        self.server = xmlrpclib.ServerProxy("http://0.0.0.0:9000/")
        print(self.server)

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.server.get_angle(joint_name)

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.server.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        return self.server.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        self.post.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.server.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.post.set_transform(effector_name, transform)


if __name__ == '__main__':
    agent = ClientAgent()
    #agent.execute_keyframes(leftBackToStand())
    #T = identity(4)
    #T[-1, 1] = 0.05
    #T[-1, 2] = 0.26
    T=[[1,0,0,0],
       [0,1,0,0.5],
       [0,0,1,0.26],
       [0,0,0,1]]
    agent.set_transform('LLeg', T)
    #agent.set_angle('LHipYawPitch', 1)