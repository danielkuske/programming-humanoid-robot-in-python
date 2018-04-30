
import os
import sys
import numpy as np
import naoqi
import motion
import time
from image_analysis import ImageAnalysis
#sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'pynao', 'naoqi'))



class GrabberAgent(object):

    def __init__(self, ip="10.0.7.14",
                 port=9559):
        self.isAlive = True
        self.count = 0
        self.positionValues = [[0, 0, 0]] * 1
        self.analyzer = ImageAnalysis()

        self.ttsProxy = naoqi.ALProxy("ALTextToSpeech", ip, port)
        self.motionProxy = naoqi.ALProxy("ALMotion", ip, port)
        self.memoryProxy = naoqi.ALProxy("ALMemory", ip, port)
        self.visionProxy = naoqi.ALProxy("RobocupVision", ip, port)

        self.motionProxy.wakeUp()
        # self.motionProxy.moveInit()
        # self.motionProxy.rest()
        self.motionProxy.angleInterpolationWithSpeed(
            ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
            [np.pi/2, 0., -np.pi / 2, -np.pi / 3, np.pi/2, 0., np.pi / 2., np.pi / 3], .2)



    def liveCycle(self):
        self.ttsProxy.say("Hello! Where's the pen?")
        while (self.isAlive):
            top_image, bad_image = self.fotoshooting()
            self.analyzer.analyzeImages(top_image, bad_image)

            if self.analyzer.penFound:
                headYaw = self.memoryProxy.getData("Device/SubDeviceList/HeadYaw/Position/Actuator/Value")
                headPitch = self.memoryProxy.getData("Device/SubDeviceList/HeadPitch/Position/Actuator/Value")
                print self.analyzer.penPosition
                x, y, z = self.analyzer.calculatePenToBody(headYaw, headPitch)
                print x,y,z

                self.analyzer.plotMask(top_image, bad_image)
                if self.analyzePositionValues(x, y, z):
                    self.count += 1
                    self.ttsProxy.say("Oh, a pen!")
                    print "pen found"
                    x, y, z = np.array(self.positionValues).mean(0)
                    z -= 100
                    if (y >= 0):
                        rho, phi = self.transformToAngles(x,y-98,z)
                        self.motionProxy.angleInterpolationWithSpeed(
                            ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'], [rho, phi - np.tan(15. / 105.), 0., 0.], .7)
                    else:
                        rho, phi = self.transformToAngles(x,y+98,z)
                        self.motionProxy.angleInterpolationWithSpeed(
                            ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'], [rho, phi  + np.tan(15. / 105.), 0., 0.], .7)


                    self.ttsProxy.say("Thats the pen number " + str(self.count) + "!")
                    #time.sleep(1)
                    self.positionValues = [[0, 0, 0]] * 1
                    self.motionProxy.angleInterpolationWithSpeed(
                        ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                        [np.pi/2, 0., -np.pi / 2, -np.pi / 3, np.pi/2, 0., np.pi / 2., np.pi / 3], .7)

            else:
                self.analyzer.plotMask(top_image, bad_image)


    def analyzePositionValues(self, x, y, z):
        self.positionValues.pop(0) # .popleft()
        self.positionValues.append([x, y, z])
        if (np.linalg.norm(np.array(self.positionValues).std(0)[1:]) < 3):
            return True
        return False

    def fotoshooting(self):
        dataBot = self.visionProxy.getBGR24Image(1)
        dataTop = self.visionProxy.getBGR24Image(0)
        imageBot = np.fromstring(dataBot, dtype=np.uint8).reshape((480, 640, 3))
        imageTop = np.fromstring(dataTop, dtype=np.uint8).reshape((480, 640, 3))
        return imageTop, imageBot

    def transformToAngles(self, x, y, z):
        r = np.sqrt(x**2 + y**2 + z**2)
        return(np.arccos(z / r) - np.pi / 2, np.arctan(y / x))



if __name__ == '__main__':
    agent = GrabberAgent()
    agent.liveCycle()

    #motionProxy = naoqi.ALProxy("ALMotion", "10.0.7.14", 9559)

    #motionProxy.wakeUp()
    #motionProxy.moveInit()
    #motionProxy.moveTo(.1, 0, 0)
    #print motionProxy.getPosition('LArm', 0, True)
    #motionProxy.positionInterpolation("LArm", 0, [[0.08824242651462555, 0.05107875168323517, -0.045162465423345566, -1.4410802125930786, 0.39402803778648376, -0.8428454995155334]], 63, [3.], True)
    #motionProxy.changePosition('Head',0, (.2, 5., 0., 0., 1., 0.), 0.2, 63)
    #motionProxy.changeTransform('LArm',0, trans, 0.2, 63)
    #print motionProxy.getTransform('LArm', 0, True)
    #time.sleep(5.)

    #agent.liveCycle()
    #while (True):
    #    bot, top = agent.fotoshooting()
    #    agent.analyzer.analyzeImages(top, bot)
    #    agent.analyzer.plot()
    #    agent.analyzer.plotMask(top, bot)
    #    time.sleep(1.)
