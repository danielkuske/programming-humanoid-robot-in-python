
import numpy as np
import naoqi
from image_analysis import ImageAnalysis



class GrabberAgent(object):

    def __init__(self, ip="10.0.7.14", port=9559):
        # Initialize class variables
        self.is_alive = True
        self.count = 0
        self.image_count = 1
        self.analyzer = ImageAnalysis()

        # Initialize initialize proxys
        self.ttsProxy = naoqi.ALProxy("ALTextToSpeech", ip, port)
        self.motionProxy = naoqi.ALProxy("ALMotion", ip, port)
        self.memoryProxy = naoqi.ALProxy("ALMemory", ip, port)
        self.visionProxy = naoqi.ALProxy("RobocupVision", ip, port)

        # Go to initial position
        self.motionProxy.wakeUp()
        self.go_to_initial_position()

    # The basic livecycle of our robot
    def live_cycle(self):
        self.ttsProxy.say("Hello! Where's the pen?")
        while (self.is_alive):
            top_image, bad_image = self.fotoshooting()
            self.analyzer.analyze_images(top_image, bad_image)
            if self.analyzer.pen_found:
                headYaw = self.memoryProxy.getData("Device/SubDeviceList/HeadYaw/Position/Actuator/Value")
                headPitch = self.memoryProxy.getData("Device/SubDeviceList/HeadPitch/Position/Actuator/Value")
                x, y, z = self.analyzer.calculate_pen_to_body(headYaw, headPitch)
                self.analyzer.plot_mask(top_image, bad_image)
                if self.analyze_position_values(x, y, z):
                    self.count += 1
                    self.ttsProxy.say("Oh, a pen!")
                    x, y, z = np.array(self.position_values).mean(0)
                    self.point_to_pen(x, y, z)
                    self.ttsProxy.say("Thats the pen number " + str(self.count) + "!")
                    self.go_to_initial_position()
            else:
                self.analyzer.plot_mask(top_image, bad_image)

    # Reset the arm positions and the saved former position values of the pen
    def go_to_initial_position(self):
        self.position_values = [[0, 0, 0]] * self.image_count
        self.motionProxy.angleInterpolationWithSpeed(
            ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
             'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
            [np.pi/2, 0., -np.pi / 2, -np.pi / 3, np.pi/2, 0., np.pi / 2., np.pi / 3], .7)

    # decide which arm to use, calculate angles for the shoulder joints and point to the pen
    def point_to_pen(self, x, y, z):
        if (y >= 0):
            rho, phi = self.transform_to_angles(x,y-98,z-100)
            self.motionProxy.angleInterpolationWithSpeed(
                ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                [rho, phi - np.tan(15. / 105.), 0., 0.],
                .7)
        else:
            rho, phi = self.transform_to_angles(x,y+98,z-100)
            self.motionProxy.angleInterpolationWithSpeed(
                ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                [rho, phi + np.tan(15. / 105.), 0., 0.],
                .7)

    # decide whether the last self.image_count many poitions of the pen are about on the same spot
    def analyze_position_values(self, x, y, z):
        self.position_values.pop(0) # .popleft()
        self.position_values.append([x, y, z])
        if (np.linalg.norm(np.array(self.position_values).std(0)[1:]) < 3):
            return True
        return False

    # make fotos with both top and bot camera
    def fotoshooting(self):
        data_bot = self.visionProxy.getBGR24Image(1)
        data_top = self.visionProxy.getBGR24Image(0)
        image_bot = np.fromstring(data_bot, dtype=np.uint8).reshape((480, 640, 3))
        image_top = np.fromstring(data_top, dtype=np.uint8).reshape((480, 640, 3))
        return image_top, image_bot

    # transform kartesian to spherical koordinates
    def transform_to_angles(self, x, y, z):
        r = np.sqrt(x**2 + y**2 + z**2)
        return(np.arccos(z / r) - np.pi / 2, np.arctan(y / x))



if __name__ == '__main__':
    agent = GrabberAgent()
    agent.live_cycle()

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

    #agent.live_cycle()
    #while (True):
    #    bot, top = agent.fotoshooting()
    #    agent.analyzer.analyze_images(top, bot)
    #    agent.analyzer.plot()
    #    agent.analyzer.plot_mask(top, bot)
    #    time.sleep(1.)
