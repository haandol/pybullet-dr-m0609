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
        p.resetBasePositionAndOrientation(self.id, [0, 0, 0], [0, 0, 0, 1])

        self.ee_index = 10
        self.numJoints = 0
        for i in range(p.getNumJoints(self.id)):
            joint_type = p.getJointInfo(self.id, i)[2]
            if joint_type == p.JOINT_PRISMATIC or joint_type == p.JOINT_REVOLUTE:
                self.numJoints += 1
        print(f"numJoints: {self.numJoints}")

        start_joint_pos = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
        for i in range(self.numJoints):
            p.resetJointState(self.id, i, start_joint_pos[i])

        self.closed = True

    def move(self):
        min_joint_positions = [
            p.getJointInfo(self.id, i)[8]  # joint lower limit
            for i in range(self.numJoints)
        ]
        max_joint_positions = [
            p.getJointInfo(self.id, i)[9]  # joint upper limit
            for i in range(self.numJoints)
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

        target_position = [1.0, 1.0, 1.0]
        target_joint_positions = p.calculateInverseKinematics(
            self.id,
            self.ee_index,
            target_position,
            lowerLimits=min_joint_positions,
            upperLimits=max_joint_positions,
            jointRanges=joint_ranges,
            restPoses=rest_poses,
        )

        for i in range(self.numJoints):
            p.setJointMotorControl2(
                bodyIndex=self.id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_joint_positions[i],
                targetVelocity=0,
                force=500,
                positionGain=0.03,
                velocityGain=1,
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
            targetPosition=0.0005,
            force=1000,
        )
        p.setJointMotorControl2(
            self.id,
            9,
            p.POSITION_CONTROL,
            targetPosition=0.0005,
            force=1000,
        )
        self.closed = True
