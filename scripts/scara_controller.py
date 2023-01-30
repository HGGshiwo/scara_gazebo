#! /usr/bin/env python
# -*- coding: utf-8 -*
import rospy
import argparse
from enum import IntEnum
from scara_gazebo.msg import Poses
from geometry_msgs.msg import Pose, Point
from scara_interface import ScaraInterface

class state(IntEnum):
    waiting = 0
    move_forth = 1
    down_forth = 2
    grasp = 3
    up_forth = 4
    move_back = 5
    down_back = 6
    release = 7
    up_back = 8

class ScaraController(ScaraInterface):

    def __init__(self, controller_name, robot_name, robot_pose, \
                    start_pose, end_pose, r1_pose, r2_pose):
        """
        创建scara控制器

        Input:  controller_name - 控制器节点名称
                robot_name - 控制器对应的机械臂名称
                robot_pose - scara机械臂所在位置
                start_pose - 开始搬运的位置
                end_pose   - 结束搬运的位置
                r1_pose    - rotation1的初始位置
                r2_pose    - rotation2的初始位置    
        """
        ScaraInterface.__init__(self, controller_name, robot_name, robot_pose, \
                                    r1_pose, r2_pose)
              
        self.start_pose = start_pose # 开始搬运的位置
        self.end_pose = end_pose # 结束搬运的位置
        self.target_loop_num = 0 # 目标循环次数
        self.cur_loop_num = 0 # 计算循环次数 
        self.cur_state = state.waiting # 当前的状态
        self.cur_action = self.wait # 当前状态执行的函数
        self.time_unit = 0.01 # 每次循环的时间单位
        self.waiting = False # 是否在等待物体到达
        
        # 状态表, next_state, next_action, duration
        self.func_tbl = [
            (state.move_forth, self.move_forth, 1.50),
            (state.down_forth, self.move_down , 0.25),
            (state.grasp     , self.grasp     , 0.05),
            (state.up_forth  , self.move_up   , 0.25),
            (state.move_back , self.move_back , 1.50),
            (state.down_back , self.move_down , 0.25),
            (state.release   , self.release   , 0.05),
            (state.up_back   , self.move_up   , 0.25),
            (state.waiting   , self.wait      , 0.00),
        ]
        
        # 订阅位置信息，注册回调
        rospy.Subscriber('/rfid_tags', Poses, self.callback)

    def wait(self):
        pass

    def move_forth(self):
        self.move_to(self.end_pose)

    def move_back(self):
        self.move_to(self.start_pose)
        
    def callback(self, poses):
        for pose in poses:
            if pose.position == self.target_pose:
                self.waiting = False
                break

    def start(self):
        """
        开始主循环
        """
        while not rospy.is_shutdown():
            if self.cur_state == state.waiting and self.waiting: # 没有物体到达target_pose，等待
                pass
            else:
                if self.cur_state == state.waiting and not self.waiting:
                    self.waiting = True # 物体到达start_pose, 第一次进入循环，重新设置waiting信号

                if self.cur_loop_num == self.target_loop_num: # 到达该状态的循环次数，则更新状态
                    self.cur_state, self.cur_action, time_cost = self.func_tbl[self.cur_state]
                    self.target_loop_num = time_cost//self.time_unit
                    self.cur_loop_num = 0
                else:
                    self.cur_action()
                    self.cur_loop_num += 1
            
            self.update()

            rospy.loginfo(self.cur_state)
            rospy.sleep(self.time_unit)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='argparse for scara_controller')
    parser.add_argument('-cn', type=str, default="scara_controller", help="name of the controller node")
    parser.add_argument('-rn', type=str, default="scara_robot1", help="name of the robot")
    parser.add_argument('-rpx', type=float, default="0.0", help="robot_pose.position.x")
    parser.add_argument('-rpy', type=float, default="0.0", help="robot_pose.position.y")
    parser.add_argument('-rpz', type=float, default="0.0", help="robot_pose.position.z")
    parser.add_argument('-spx', type=float, default="1.5", help="start_pose.position.x")
    parser.add_argument('-spy', type=float, default="0.0", help="start_pose.position.y")
    parser.add_argument('-spz', type=float, default="0.0", help="start_pose.position.z")
    parser.add_argument('-epx', type=float, default="0.0", help="end_pose.position.x")
    parser.add_argument('-epy', type=float, default="1.5", help="robot_pose.position.y")
    parser.add_argument('-epz', type=float, default="0.0", help="robot_pose.position.z")
    parser.add_argument('-r1p', type=float, default="-0.78", help="rotation1_joint init angle")
    parser.add_argument('-r2p', type=float, default="2.1", help="rotation2_joint init angle")
    
    myargv = rospy.myargv()
    args = parser.parse_args(myargv[1:])
    
    scara_controller = \
        ScaraController(
            controller_name=args.cn, 
            robot_name=args.rn, 
            robot_pose=Pose(position=Point(args.rpx, args.rpy, args.rpz)),
            start_pose=Pose(position=Point(args.spx, args.spy, args.spz)),
            end_pose=Pose(position=Point(args.epx, args.epy, args.epz)),
            r1_pose=args.r1p,
            r2_pose=args.r2p
        )
    scara_controller.start()
