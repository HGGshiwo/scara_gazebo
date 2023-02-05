#! /usr/bin/env python
# -*- coding: utf-8 -*
import rospy
import numpy as np
from joint import Joint

class ScaraInterface:

    def __init__(self, controller_name, robot_name, robot_pose, r1_pose, r2_pose):
        """
        创建scara控制器

        Input:  controller_name - 控制器节点名称
                robot_name - 控制器对应的机械臂名称
                robot_pose - scara机械臂所在位置
                r1_pose    - rotation1的初始位置
                r2_pose    - rotation2的初始位置    
        """
        rospy.init_node(controller_name)
        
        # 关节控制的参数
        self.kps = [80, 0.8]
        self.kvs = [64, 0.2]
        self.GRIPPER_UP_EFFORT = 0.001
        self.GRIPPER_DOWN_EFFORT = 0.0
        self.GRIPPER_GRASP_EFFORT = 0.002
        self.GRIPPER_RELEASE_EFFORT = 0.0001
        
        self.robot_name = robot_name # 机器人名字
        self.robot_pose = robot_pose # 机器人所在的位置
        
        # 初始化joint信息
        self.joints={
            # rotation joint
            "rotation1": Joint("{}::rotation1".format(self.robot_name), init_pose=r1_pose),
            "rotation2": Joint("{}::rotation2".format(self.robot_name), init_pose=r2_pose),
            # joint control gripper up
            "gripper": Joint("{}::gripper_joint".format(self.robot_name), init_effort=self.GRIPPER_UP_EFFORT, duration=0.25),
            # joint control gripper fingers
            "finger1": Joint("{}::finger1_joint".format(self.robot_name), init_effort=self.GRIPPER_RELEASE_EFFORT, duration=0.25),
            "finger2": Joint("{}::finger2_joint".format(self.robot_name), init_effort=-self.GRIPPER_RELEASE_EFFORT, duration=0.25),
            "finger3": Joint("{}::finger3_joint".format(self.robot_name), init_effort=-self.GRIPPER_RELEASE_EFFORT, duration=0.25),
            "finger4": Joint("{}::finger4_joint".format(self.robot_name), init_effort=self.GRIPPER_RELEASE_EFFORT, duration=0.25),
        }

    
    def move_to(self, pose):
        """
        移动gripper到指定的pose

        Input  pose - 移动的目标
        """
        # 计算相对坐标, 不清楚两个为什么是反着减
        y = self.robot_pose.position.x - pose.position.x
        x = self.robot_pose.position.y - pose.position.y
        dist_square = x*x + y*y # 目标到机器人中心的距离平方
        # 余弦定理计算出两个joint的转动角度, scara和中心连接的手臂长度为1, 另一个手臂长度为0.8
        angles = [
            np.arctan(np.divide(y,x)) - np.arccos((0.36 + dist_square)/(2*np.sqrt(dist_square))),
            np.pi - np.arccos((1.64 - dist_square)/1.6)
        ]
        # add robust to this inverse kinematics
        if np.isnan(angles).any():
            angles = [np.arctan(y/x), 0]
               
         # 发布joint需要旋转的角度
        for i,name in enumerate(["rotation1", "rotation2"]):
            pose_err = angles[i] - self.joints[name].cur_pose
            effort = self.kps[i] * pose_err - self.kvs[i] * self.joints[name].cur_rate
            effort = round(effort, 4)
            self.joints[name].set_effort(effort)
            self.joints[name].publish()

    def move_down(self):
        """
        向下移动gripper
        """
        self.joints["gripper"].set_effort(self.GRIPPER_DOWN_EFFORT)
        pass
    
    def move_up(self):
        """
        向上移动gripper
        """
        self.joints["gripper"].set_effort(self.GRIPPER_UP_EFFORT)

    def grasp(self):
        self.joints["finger1"].set_effort(-self.GRIPPER_GRASP_EFFORT)
        self.joints["finger2"].set_effort(self.GRIPPER_GRASP_EFFORT)
        self.joints["finger3"].set_effort(self.GRIPPER_GRASP_EFFORT)
        self.joints["finger4"].set_effort(-self.GRIPPER_GRASP_EFFORT)

    def release(self):
        self.joints["finger1"].set_effort(self.GRIPPER_RELEASE_EFFORT)
        self.joints["finger2"].set_effort(-self.GRIPPER_RELEASE_EFFORT)
        self.joints["finger3"].set_effort(-self.GRIPPER_RELEASE_EFFORT)
        self.joints["finger4"].set_effort(self.GRIPPER_RELEASE_EFFORT)

    def update(self):
        """
        更新rotation_joint的位置, 发布gripper_joint和finger_joint的力
        """
        # 更新joint目前的位置
        for joint_name in ["rotation1", "rotation2"]:
            self.joints[joint_name].update_state()
        
        # 持续发布joint effort
        for joint_name in ["gripper", "finger1", "finger2", "finger3", "finger4"]:
            self.joints[joint_name].publish()