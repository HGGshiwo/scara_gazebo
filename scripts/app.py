#! /usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import json
import argparse
from scara_interface import ScaraInterface
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from enum import IntEnum

# '{"car_id":"001","arm_id":"scara_robot1","cargo_id":"001","destination":1}'

class state(IntEnum):
    wait1 = 0
    move1 = 1
    down1 = 2
    grasp = 3
    up1 = 4
    move2 = 5
    down2 = 6
    release = 7
    up2 = 8
    wait2 = 9

class App(ScaraInterface):
    def __init__(self, controller_name, robot_name, robot_pose, start_pose, end_pose, r1_pose, r2_pose):
        ScaraInterface.__init__(self, controller_name, robot_name, robot_pose, r1_pose, r2_pose)
        
        self.start_pose = start_pose
        self.end_pose = end_pose
        
        self.publisher = rospy.Publisher("load_done", String, queue_size=10)
        
        self.car_arrive = False # 小车是否到
        self.schedule_done = False # 是否调度完成
        self.target_loop_num = 1 # 目标循环次数
        self.cur_loop_num = 0 # 计算循环次数 
        self.cur_state = state.wait1 # 当前的状态
        self.cur_action = self.wait1 # 当前状态执行的函数
        self.time_unit = 0.01 # 每次循环的时间单位

        self.func_tbl = {
            state.wait1: (state.move1, self.move1    ,150),
            state.move1: (state.down1, self.move_down, 25),
            state.down1: (state.grasp, self.grasp    ,  5),
            state.grasp: (state.up1  , self.move_up  , 25),
            state.up1:   (state.move2, self.move2    ,150),
            state.move2: (state.wait2, self.wait2     ,  1),
            state.wait2: (state.down2, self.move_down, 25),
            state.down2: (state.release,self.release ,  5),
            state.release: (state.up2, self.up2      , 25),
            state.up2:   (state.wait1, self.wait1     , 1),
        }

    def move1(self):
        self.move_to(self.start_pose)
        
    def move2(self):
        self.move_to(self.end_pose)

    def wait1(self):
        if self.schedule_done:
            self.cargo_arrive = False
            self.cur_loop_num = 0
        else:
            self.cur_loop_num -= 1

    def wait2(self):
        if self.car_arrive:
            self.car_arrive = False
            self.cur_loop_num = 0
        else:
            self.cur_loop_num -= 1

    def up2(self):
        self.move_up()

        if self.cur_loop_num == 0:
            # 发布一条信息，表示机械臂完成了搬运
            msg = {"arm_id": self.robot_name}
            msg = json.dumps(msg)
            self.publisher.publish(msg)
    
    def schedule_done_callback(self, msg):
        """
        定义回调函数, 在schedule_done的时候调用 
        """
        data = msg.data
        rospy.loginfo(data)
        data = json.loads(data)
        
        if data["arm_id"] != self.robot_name:
            return
        self.schedule_done = True

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.cur_state)
            if self.cur_loop_num == self.target_loop_num: # 到达该状态的循环次数，则更新状态
                self.cur_state, self.cur_action, self.target_loop_num = self.func_tbl[self.cur_state]
                self.cur_loop_num = 0
            else:
                self.cur_action()
                self.cur_loop_num += 1
            
            self.update()

            rospy.sleep(self.time_unit)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='argparse for scara_controller')
    parser.add_argument('-cn', type=str, default="scara_controller", help="name of the controller node")
    parser.add_argument('-rn', type=str, default="scara_robot1", help="name of the robot")
    parser.add_argument('-rpx', type=float, default="0.0", help="robot_pose.position.x")
    parser.add_argument('-rpy', type=float, default="0.0", help="robot_pose.position.y")
    parser.add_argument('-rpz', type=float, default="0.0", help="robot_pose.position.z")
    parser.add_argument('-spx', type=float, default="0.0", help="start_pose.position.x")
    parser.add_argument('-spy', type=float, default="0.0", help="start_pose.position.y")
    parser.add_argument('-spz', type=float, default="0.0", help="start_pose.position.z")
    parser.add_argument('-epx', type=float, default="0.0", help="end_pose.position.x")
    parser.add_argument('-epy', type=float, default="0.0", help="robot_pose.position.y")
    parser.add_argument('-epz', type=float, default="0.0", help="robot_pose.position.z")
    parser.add_argument('-r1p', type=float, default="0.0", help="rotation1_joint init angle")
    parser.add_argument('-r2p', type=float, default="0.0", help="rotation2_joint init angle")
    
    myargv = rospy.myargv()
    args = parser.parse_args(myargv[1:])
    
    app = \
        App(
            controller_name=args.cn, 
            robot_name=args.rn, 
            robot_pose=Pose(position=Point(args.rpx, args.rpy, args.rpz)),
            start_pose=Pose(position=Point(args.spx, args.spy, args.spz)),
            end_pose=Pose(position=Point(args.epx, args.epy, args.epz)),
            r1_pose=args.r1p,
            r2_pose=args.r2p
        )
    
    rospy.logdebug(app.start_pose.position.x)
    rospy.Subscriber("schedule_done", String, app.schedule_done_callback)
    app.run()