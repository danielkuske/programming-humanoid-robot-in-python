'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
from SimpleXMLRPCServer import SimpleXMLRPCServer
import xmlrpclib
from agent_server import ServerAgent
from threading import Thread
from keyframes import leftBackToStand

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''

    def __init__(self, obj):
        self.obj = obj
        self.proxy = weakref.proxy(obj)


    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        Thread(target=self.obj.server.execute_keyframes, args=(keyframes))

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        Thread(target=self.obj.server.set_transform, args=(effector_name, transform))


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE

    def __init__(self):
        self.post = PostHandler(self)
        #self.server = client.ServerProxy("http://localhost:8004/")
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
        self.server.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.server.get_transform()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.post.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ClientAgent()
    agent.execute_keyframes(leftBackToStand())