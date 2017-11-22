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


from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        n = len(keyframes.names)
        for i in xrange(0, n-1):
            name = keyframes.names[i]
            time = keyframes.times[i]
            keys = keyframes.keys[i]

            #find right key:
            j = 0
            for index, t in enumerate(time):
                if (t > perception.time):
                    j = index
                    break
            bezierStartPoint = keys[j-1]
            bezierEndPoint = keys[j]



        #( (1 - t ) ( (1 - t ) ( (1 - t )x_0 + tx_1 )+t ( (
        #    1 - t )x_1 + tx_2 ) )+t ( (1 - t ) ( (
        #    1 - t )x_1 + tx_2 )+t ( (1 - t )x_2 + tx_3 ) ),  (
        #    1 - t ) ( (1 - t ) ( (1 - t )y_0 + ty_1 )+t ( (
        #    1 - t )y_1 + ty_2 ) )+t ( (1 - t ) ( (
        #    1 - t )y_1 + ty_2 )+t ( (1 - t )y_2 + ty_3 ) ) )

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
