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
        self.closed = True

    def open_gripper(self):
        p.setJointMotorControl2(
            self.id,
            8,
            p.POSITION_CONTROL,
            targetPosition=0.04,
            force=1000,
        )
        p.setJointMotorControl2(
            self.id,
            9,
            p.POSITION_CONTROL,
            targetPosition=0.04,
            force=1000,
        )
        self.closed = False

    def close_gripper(self):
        p.setJointMotorControl2(
            self.id,
            8,
            p.POSITION_CONTROL,
            targetPosition=0.005,
            force=1000,
        )
        p.setJointMotorControl2(
            self.id,
            9,
            p.POSITION_CONTROL,
            targetPosition=0.005,
            force=1000,
        )
        self.closed = True
