'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
from os import listdir
import pickle

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)

        ROBOT_POSE_DATA_DIR = '../joint_control/robot_pose_data'
        self.classes = listdir(ROBOT_POSE_DATA_DIR)
        self.posture = 'Stand'
        self.posture_classifier = pickle.load(open('../joint_control/robot_pose.pkl'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        posture = self.recognize_posture(perception)
        if posture != self.posture:
            print posture
            self.posture = posture
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        #where 'AngleX' and 'AngleY' are body angle (e.g. Perception.imu) and others are joint angles
        data = [perception.joint['LHipYawPitch'], perception.joint['LHipRoll'], perception.joint['LHipPitch'],
                perception.joint['LKneePitch'], perception.joint['RHipYawPitch'], perception.joint['RHipRoll'],
                perception.joint['RHipPitch'], perception.joint['RKneePitch'], perception.imu[0], perception.imu[1]]
        posture = self.classes[self.posture_classifier.predict([data])[0]]
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
