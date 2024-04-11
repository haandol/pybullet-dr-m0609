import math

import pybullet as p


class Robot(object):
    def __init__(self) -> None:
        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, math.pi / 2])

        self.id = p.loadURDF(
            "doosan_robot/m0609.urdf",
            startPos,
            startOrientation,
            useFixedBase=True,
        )
