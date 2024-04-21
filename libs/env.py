import time
import math
import queue
import random
import logging
import multiprocessing as mp

import pybullet as p
import pybullet_data

from libs.robot import Robot
from libs.messages.commands import MoveToXYZ
from libs.messages.events import (
    EnvironmentReady,
    RobotMoved,
    TaskCompleted,
)

logger = logging.getLogger("env")


class Environment(mp.Process):
    def __init__(self, robot: Robot, cmd_q: mp.Queue, evt_q: mp.Queue) -> None:
        super(Environment, self).__init__()
        self.robot = robot
        self.cmd_q = cmd_q
        self.evt_q = evt_q

    def load_red_box(self, urdf_path: str = "assets/ycb_assets/003_cracker_box.urdf"):
        start_pos = [0.5, 0.5, 0.1]
        start_orientation = p.getQuaternionFromEuler(
            [0.0, 0.0, random.uniform(-math.pi, math.pi)]
        )
        boxId = p.loadURDF(
            urdf_path,
            start_pos,
            start_orientation,
            useFixedBase=False,
            globalScaling=0.08,
        )
        return boxId

    def load_blue_can(
        self, urdf_path: str = "assets/ycb_assets/002_master_chef_can.urdf"
    ):
        start_pos = [-0.5, 0.5, 0.1]
        start_orientation = p.getQuaternionFromEuler(
            [0.0, 0.0, random.uniform(-math.pi, math.pi)]
        )

        canId = p.loadURDF(
            urdf_path,
            start_pos,
            start_orientation,
            useFixedBase=False,
            globalScaling=0.08,
        )
        return canId

    def adjust_camera(self):
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=200,
            cameraPitch=-40,
            cameraTargetPosition=[0.0, 1.0, 1.2],
        )

    def load(self):
        p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -9.807)
        p.loadURDF("plane.urdf")

        self.load_red_box()
        self.load_blue_can()
        self.adjust_camera()
        self.robot.load()
        self.robot.open_gripper()

    def update(self):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)  # 240Hz

    def run(self):
        self.load()
        logger.info("Environment loaded.")
        self.evt_q.put(EnvironmentReady())

        # consume commands
        while True:
            try:
                cmd = self.cmd_q.get(block=False)
                logger.info(f"cmd: {cmd}")
                if isinstance(cmd, MoveToXYZ):
                    joint_positions = self.robot.move(cmd.xyz)
                    for _ in range(100):
                        self.update()
                    self.evt_q.put(RobotMoved(joint_positions=joint_positions))
                else:
                    self.evt_q.put(TaskCompleted())
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(e)
                raise e
            finally:
                self.update()
