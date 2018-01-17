'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''

import numpy as np
from pid import PIDAgent
from keyframes import leftBellyToStand
from keyframes import leftBackToStand


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        if target_joints != 0:
            self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}

        if (self.start_time == -1):
            self.start_time = perception.time
        keyframe_exec_time = perception.time - self.start_time

        for i, name in enumerate(keyframes[0]):
            if not name in perception.joint:
                continue

            time = keyframes[1][i]
            keys = keyframes[2][i]

            #find right key:
            j = -1
            for index, t in enumerate(time):
                if (t > keyframe_exec_time):
                    j = index
                    break
            if j == -1:
                self.keyframes = ([],[],[])
                self.start=-1
                return 0
#                target_joints[name] = 0
#                continue

            endHandleDTime = keys[j][1][1]
            endHandleDAngle = keys[j][1][2]
            bezierEnd = (time[j], keys[j][0])
            bezierEndHandle =  np.add(bezierEnd, (endHandleDTime, endHandleDAngle))

            if (j > 0):
                startHandleDTime =  keys[j-1][2][1]
                startHandleDAngle = keys[j-1][2][2]
                bezierStart = (time[j-1], keys[j-1][0])
                bezierStartHandle =  np.add(bezierStart, (startHandleDTime, startHandleDAngle))
            else:
                startHandleDTime = - endHandleDTime
                startHandleDAngle = perception.joint[name]
                bezierStart = (0, perception.joint[name])
                bezierStartHandle = np.add(bezierStart, (startHandleDTime, startHandleDAngle))

            root = self.root_of_bezier_polynomial(
                bezierStart[0],
                bezierStartHandle[0],
                bezierEndHandle[0],
                bezierEnd[0],
                keyframe_exec_time)

            target_angle = self.value_of_bezier_polynomial(
                bezierStart[1],
                bezierStartHandle[1],
                bezierEndHandle[1],
                bezierEnd[1],
                root)

            #print name + ':' + str(target_angle)
            target_joints[name] = target_angle

        return target_joints

    @staticmethod
    def root_of_bezier_polynomial(x0, x1, x2, x3, t):
        roots = np.roots([-   x0 + 3 * x1 - 3 * x2 + x3,
                                 3 * x0 - 6 * x1 + 3 * x2,
                               - 3 * x0 + 3 * x1,
                                -t + x0])
        i = []
        for r in roots:
            if np.isreal(r) and 0 <= np.real(r) <= 1:
                i.append(r)
        if len(i) > 1:
            print 'shiiit too many'
            print(i)
        for r in roots:
            if np.isreal(r) and 0 <= np.real(r) <= 1:
                return np.real(r)
        print 'shiiit not enough'
        return 0

    @staticmethod
    def value_of_bezier_polynomial(x0, x1, x2, x3, val):
        return np.polyval([-     x0 + 3 * x1 - 3 * x2 + x3,
                             3 * x0 - 6 * x1 + 3 * x2,
                           - 3 * x0 + 3 * x1,
                                 x0],
                          val)

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
