
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from angle_interpolation import AngleInterpolationAgent
from enum import Enum

class State(Enum):
    INIT=0  # Hinsetzen (keyframe)
    WATCH=1 # Kopf drehen (keyframe), bild analysieren
    GRABB=2 # Backward kinematics berechnen und ausfuehren (Hand ist vor Stift, bereit zum zugreifen)
    DROP=3  # Zugreifen (keyframe), drop (keyframe)

#   Stift bauen
#   Bildanalyse
#   keyframes
#       Hinsetzen
#       Kopf drehen
#       Zugreifen
#       drop
#   Backward Kinematics (rechter arm)
class GrabberAgent(AngleInterpolationAgent):

    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(GrabberAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.state = State.INIT

    def think(self, perception):
        if self.state == State.INIT:
            #hinsetzen keyframe
            self.State = State.WATCH
        elif self.state == State.WATCH:
            # analyse Camera
            # move head
            grabPosition = False
            if grabPosition:
                # calculate backward kinematics for one arm
                self.state = State.GRABB
        elif self.state == State.GRABB:
            # perform grab action that has been calculated before
            # if pen before hand
            self.state = State.DROP
        else:
            # perform drop animation saved in a keyframe
            #if finished
            self.state = State.WATCH
        return super(GrabberAgent, self).think(perception)


if __name__ == '__main__':
    agent = GrabberAgent()
    agent.run()
