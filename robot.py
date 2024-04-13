import math

import numpy as np
import pybullet as p


class Robot(object):
    def __init__(self) -> None:
        start_pos = [0, 0, 0]
        start_orientation = p.getQuaternionFromEuler([0, 0, math.pi / 2])

        self.id = p.loadURDF(
            "doosan_robot/m0609.urdf",
            start_pos,
            start_orientation,
            useFixedBase=True,
        )

        p.setJointMotorControlArray(
            self.id, range(7), p.VELOCITY_CONTROL, forces=[0] * 7
        )

        self.closed = True

    def move(self):
        min_joint_positions = [
            p.getJointInfo(self.id, i)[8]
            for i in range(p.getNumJoints(self.id))
            if p.getJointInfo(self.id, i)[2] == p.JOINT_PRISMATIC
            or p.getJointInfo(self.id, i)[2] == p.JOINT_REVOLUTE
        ]
        max_joint_positions = [
            p.getJointInfo(self.id, i)[9]
            for i in range(p.getNumJoints(self.id))
            if p.getJointInfo(self.id, i)[2] == p.JOINT_PRISMATIC
            or p.getJointInfo(self.id, i)[2] == p.JOINT_REVOLUTE
        ]
        joint_ranges = [
            abs(max_joint_position - min_joint_position)
            for min_joint_position, max_joint_position in zip(
                min_joint_positions, max_joint_positions
            )
        ]
        rest_poses = list(
            (np.array(max_joint_positions) + np.array(min_joint_positions)) / 2
        )

        target_position = [-1.0, 1.0, 1.0]
        target_joint_positions = p.calculateInverseKinematics(
            self.id,
            10,
            target_position,
            jointRanges=joint_ranges,
            restPoses=rest_poses,
            maxNumIterations=500,
        )

        p.setJointMotorControlArray(
            self.id,
            range(7),
            p.POSITION_CONTROL,
            targetPositions=target_joint_positions[:7],
            forces=[1] * 7,
        )

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
