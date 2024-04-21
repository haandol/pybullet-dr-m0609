import math
import logging

import pybullet as p

logger = logging.getLogger("robot")


class Robot(object):
    def __init__(
        self,
        urdf_path: str,
        ee_index: int,
        ee_left_finger_index: int,
        ee_right_finger_index: int,
    ) -> None:
        self.urdf_path = urdf_path
        self.id = None
        self.ee_index = ee_index
        self.ee_left_finger_index = ee_left_finger_index
        self.ee_right_finger_index = ee_right_finger_index
        self.closed = True

    def load(self):
        start_pos = [0, 0, 0]
        start_orientation = p.getQuaternionFromEuler([0, 0, math.pi / 2])

        self.id = p.loadURDF(
            self.urdf_path,
            start_pos,
            start_orientation,
            useFixedBase=True,
        )

        self.numJoints = 0
        for i in range(p.getNumJoints(self.id)):
            joint_type = p.getJointInfo(self.id, i)[2]
            if joint_type == p.JOINT_PRISMATIC or joint_type == p.JOINT_REVOLUTE:
                self.numJoints += 1
        logger.info(f"numJoints: {self.numJoints}")
        self.reset()

    def reset(self):
        self.open_gripper()

        start_joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.04]
        i = 0
        for j in range(p.getNumJoints(self.id)):
            joint_type = p.getJointInfo(self.id, j)[2]
            if joint_type == p.JOINT_PRISMATIC or joint_type == p.JOINT_REVOLUTE:
                p.resetJointState(self.id, j, start_joint_pos[i])
                i += 1

    def get_current_ee_position(self) -> list[float]:
        return p.getLinkState(self.id, self.ee_index)[0]

    def move(self, target_ee_position: list[float]) -> tuple:
        logger.info("Calculating inverse kinematics...")
        target_joint_positions = p.calculateInverseKinematics(
            self.id,
            self.ee_index,
            target_ee_position,
            maxNumIterations=800,
        )

        # draw a line from the current end effector position to the target position
        current_ee_position = self.get_current_ee_position()
        p.addUserDebugLine(
            current_ee_position,
            target_ee_position,
            [1, 0, 1],  # purple
            5,  # line width
            30,  # life time
        )
        logger.info(
            f"Move arm from {current_ee_position} to {target_ee_position} with {target_joint_positions}"
        )

        p.setJointMotorControlArray(
            self.id,
            range(self.numJoints),
            p.POSITION_CONTROL,
            targetPositions=target_joint_positions[: self.numJoints],
            forces=[1000] * self.numJoints,
        )

        return target_joint_positions

    def open_gripper(self):
        p.setJointMotorControl2(
            self.id,
            self.ee_left_finger_index,
            p.POSITION_CONTROL,
            targetPosition=0.04,
            force=1000,
        )
        p.setJointMotorControl2(
            self.id,
            self.ee_right_finger_index,
            p.POSITION_CONTROL,
            targetPosition=0.04,
            force=1000,
        )
        self.closed = False

    def close_gripper(self):
        p.setJointMotorControl2(
            self.id,
            self.ee_left_finger_index,
            p.POSITION_CONTROL,
            targetPosition=0.0005,
            force=1000,
        )
        p.setJointMotorControl2(
            self.id,
            self.ee_right_finger_index,
            p.POSITION_CONTROL,
            targetPosition=0.0005,
            force=1000,
        )
        self.closed = True


class RobotFactory(object):
    @staticmethod
    def create(robot_type: str) -> Robot:
        if robot_type == "m0609":
            return Robot(
                "assets/doosan_robot/m0609.urdf",
                ee_index=10,
                ee_left_finger_index=8,
                ee_right_finger_index=9,
            )
        elif robot_type == "franka":
            return Robot(
                "assets/franka_robot/panda.urdf",
                ee_index=11,
                ee_left_finger_index=9,
                ee_right_finger_index=10,
            )
        else:
            raise ValueError("Invalid robot type")
