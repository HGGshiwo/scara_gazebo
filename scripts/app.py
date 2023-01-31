#! /usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import json
import argparse
from scara_interface import ScaraInterface
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from enum import IntEnum

# config = {
#     "scara_robot1": {
#                 "robot_pose": Pose(position=Point(0,0,0)), 
#                 "r1_pose": Pose(position=Point(1.5,0,0)), 
#                 "r2_pose": Pose(position=Point(0,1.5,0))
#             }
# }

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
        
        self.car_arrive = True # 小车是否到
        self.target_loop_num = 0 # 目标循环次数
        self.cur_loop_num = 0 # 计算循环次数 
        self.cur_state = state.wait1 # 当前的状态
        self.cur_action = self.wait # 当前状态执行的函数
        self.time_unit = 0.01 # 每次循环的时间单位

        self.func_tbl = {
            state.wait1: (state.move1, self.move1     , 1.50),
            state.move1:   (state.down1, self.move_down , 0.25),
            state.down1:   (state.grasp, self.grasp     , 0.05),
            state.grasp:  (state.up1  , self.move_up   , 0.25),
            state.up1:    (state.move2 , self.move2,     1.50),
            state.move2:  (state.wait2, self.wait    , 0.00),
            state.wait2:  (state.down2 , self.move_down, 0.25),
            state.down2:  (state.release, self.release , 0.05),
            state.release: (state.up2   , self.move_up  , 0.25),
            state.up2:     (state.wait1   , self.wait  , 0.25),
        }

    def move1(self):
        self.move_to(self.start_pose)
        
    def move2(self):
        self.move_to(self.end_pose)

    def wait(self):
        pass

    def schedule_done(self, msg):
        """
        定义回调函数, 在schedule_done的时候调用 
        """
        data = msg.data
        rospy.loginfo(data)
        data = json.loads(data)
        
        if data["arm_id"] != self.robot_name:
            return
        rospy.loginfo(data["arm_id"])
        while not rospy.is_shutdown():
            rospy.loginfo(self.cur_state)
            if self.cur_state == state.wait1:
                if not self.car_arrive:
                    continue
                else:
                    self.car_arrive = False
            if self.cur_loop_num == self.target_loop_num: # 到达该状态的循环次数，则更新状态
                self.cur_state, self.cur_action, time_cost = self.func_tbl[self.cur_state]
                self.target_loop_num = time_cost//self.time_unit
                self.cur_loop_num = 0
            else:
                self.cur_action()
                self.cur_loop_num += 1
            
            self.update()

            rospy.sleep(self.time_unit)


        # 发布一条信息，表示机械臂完成了搬运
        msg = {"arm_id": self.robot_name}
        msg = json.dumps(msg)
        self.publisher.publish(msg)
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='argparse for scara_controller')
    parser.add_argument('-cn', type=str, default="scara_controller", help="name of the controller node")
    parser.add_argument('-rn', type=str, default="scara_robot1", help="name of the robot")
    parser.add_argument('-rpx', type=float, default="0.0", help="robot_pose.position.x")
    parser.add_argument('-rpy', type=float, default="0.0", help="robot_pose.position.y")
    parser.add_argument('-rpz', type=float, default="0.0", help="robot_pose.position.z")
    parser.add_argument('-spx', type=float, default="0.0", help="start_pose.position.x")
    parser.add_argument('-spy', type=float, default="1.5", help="start_pose.position.y")
    parser.add_argument('-spz', type=float, default="0.0", help="start_pose.position.z")
    parser.add_argument('-epx', type=float, default="1.5", help="end_pose.position.x")
    parser.add_argument('-epy', type=float, default="0.0", help="robot_pose.position.y")
    parser.add_argument('-epz', type=float, default="0.0", help="robot_pose.position.z")
    parser.add_argument('-r1p', type=float, default="-0.78", help="rotation1_joint init angle")
    parser.add_argument('-r2p', type=float, default="2.1", help="rotation2_joint init angle")
    
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
    rospy.Subscriber("schedule_done", String, app.schedule_done)
    rospy.spin() # 这里是阻塞函数，等待callback被调用