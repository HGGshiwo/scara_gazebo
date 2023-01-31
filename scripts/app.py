#! /usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import json
from scara_interface import ScaraInterface
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from enum import IntEnum

config = {
    "001": {"robot_pose": Pose(), "r1_pose": Pose(), "r2_pose": Pose()}
}

class state(IntEnum):
    waiting = 0
    move1 = 1
    down1 = 2
    grasp = 3
    up1 = 4
    move2 = 5
    down2 = 6
    release = 7
    up2 = 8

class App(ScaraInterface):
    def __init__(self, controller_name, robot_name):
        robot_pose = config[robot_name]["robot_pose"]
        r1_pose = config[robot_name]["r1_pose"]
        r2_pose = config[robot_name]["r2_pose"]

        ScaraInterface.__init__(controller_name, robot_name, robot_pose, r1_pose, r2_pose)
        self.publisher = rospy.Publisher("load_done", String)
        
        self.car_arrive = False # 小车是否到
        self.target_loop_num = 0 # 目标循环次数
        self.cur_loop_num = 0 # 计算循环次数 
        self.cur_state = state.waiting # 当前的状态
        self.cur_action = self.wait # 当前状态执行的函数
        self.time_unit = 0.01 # 每次循环的时间单位

        self.schedule_done_func_tbl = [
            (state.move1, self.move1     , 1.50),
            (state.down1, self.move_down , 0.25),
            (state.grasp, self.grasp     , 0.05),
            (state.up1  , self.move_up   , 0.25),
            (state.move2 , self.move2,     1.50),
            (state.waiting, self.wait    , 0.00),
            (state.down2 , self.move_down, 0.25),
            (state.release, self.release , 0.05),
            (state.up2   , self.move_up  , 0.25),
        ]

    def move1(self):
        self.move_to(self.r1_pose)
        
    def move2(self):
        self.move_to(self.r2_pose)

        
    def schedule_done(self, data):
        """
        定义回调函数, 在schedule_done的时候调用 
        """
        data = json.loads(data)
        if data["arm_id"] != self.robot_name:
            return
        
        while not rospy.is_shutdown():
            if self.cur_state == state.waiting:
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
    app = App("controller", "scara_robot1")
    rospy.Subscriber("schedule_done", String, app.schedule_done)
    rospy.spin() # 这里是阻塞函数，等待callback被调用