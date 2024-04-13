import sys
import time
import math
import random
import signal

import pybullet as p
import pybullet_data

from robot import Robot


def disconnect(sig, frame):
    print("Disconnecting...")
    p.disconnect()
    print("Disconnected.")
    sys.exit(0)


signal.signal(signal.SIGINT, disconnect)


class Environment(object):
    def __init__(self) -> None:
        p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -9.807)
        p.loadURDF("plane.urdf")
        self.robot = None

    def load_red_box(self):
        start_pos = [random.uniform(-0.2, 0.2), random.uniform(0.4, 0.8), 0.1]
        start_orientation = p.getQuaternionFromEuler(
            [0.0, 0.0, random.uniform(-math.pi, math.pi)]
        )
        boxId = p.loadURDF(
            "ycb_assets/003_cracker_box.urdf",
            start_pos,
            start_orientation,
            useFixedBase=False,
            globalScaling=0.08,
        )
        return boxId

    def load_blue_can(self):
        start_pos = [0.3, random.uniform(0.2, 0.4), 0.1]
        start_orientation = p.getQuaternionFromEuler(
            [0.0, 0.0, random.uniform(-math.pi, math.pi)]
        )

        canId = p.loadURDF(
            "ycb_assets/002_master_chef_can.urdf",
            start_pos,
            start_orientation,
            useFixedBase=False,
            globalScaling=0.08,
        )
        return canId

    def adjust_camera(self):
        p.resetDebugVisualizerCamera(
            cameraDistance=0.9,
            cameraYaw=225,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0.6, 0.3],
        )

    def load(self):
        self.adjust_camera()
        self.load_red_box()
        self.load_blue_can()
        self.robot = Robot()

        self.robot.move()
        self.robot.open_gripper()

    def update(self):
        time.sleep(1.0 / 240.0)
        p.stepSimulation()


if __name__ == "__main__":
    env = Environment()
    env.load()

    while True:
        env.update()
