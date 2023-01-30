import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest, ApplyJointEffort, ApplyJointEffortRequest

class Joint:

    def __init__(self, joint_name, init_pose=0, init_rate=0, init_effort=0, duration=0.01):
        """
        Joint对象, 提供查询joint状态和发布effort的方法 

        Input:  joint_name  - 完整的命名
                init_pose   - float, 初始的角度位置 
                init_rate   - float, 初始的角速度 
                init_effort - float, 初始的力
                duration    - float, 作用力持续的时间
        """
        rospy.loginfo("{} start...".format(joint_name))
        self.joint_name = joint_name # 完整的命名
        self.cur_pose = init_pose # 当前转动的角度
        self.cur_rate = init_rate # 当前的速度
        self.cur_effort = init_effort # 当前需要施加的力
        # 配置gazebo关于获取joint信息的代理
        rospy.loginfo("waiting /gazebo/get_joint_properties to start....")
        rospy.wait_for_service("/gazebo/get_joint_properties")
        self.property_msg = GetJointPropertiesRequest() # 获得joint信息的msg
        self.property_msg.joint_name = self.joint_name 
        self.get_property_proxy = rospy.ServiceProxy( # 获取joint状态的代理
            name="/gazebo/get_joint_properties", 
            service_class=GetJointProperties
        )
        # 配置gazebo关于设置joint动作的代理
        rospy.loginfo("waiting /gazebo/apply_joint_effort to start....")
        rospy.wait_for_service("/gazebo/apply_joint_effort")
        self.apply_effort_msg = ApplyJointEffortRequest() # 设置joint操作的msg
        self.apply_effort_msg.joint_name = self.joint_name
        self.apply_effort_msg.start_time = rospy.Time()
        self.apply_effort_msg.duration = rospy.Duration(0, int(duration*1e9))
        self.apply_effort_proxy = rospy.ServiceProxy( # 控制joint的代理
            name="/gazebo/apply_joint_effort", 
            service_class=ApplyJointEffort
        )

    def update_state(self):
        """
        获取joint当前的位置
        """
        response = self.get_property_proxy.call(self.property_msg)
        self.cur_pose = response.position[0] # 猜测是一个角度
        self.cur_rate = response.rate[0]

    def set_effort(self, effort):
        """
        设置施加在关节上的力

        Input:  effort - 施加的力大小
        """
        self.cur_effort = effort

    def publish(self):
        """
        向gazebo发布施加在关节上的力
        """
        self.apply_effort_msg.effort = self.cur_effort
        response = self.apply_effort_proxy.call(self.apply_effort_msg)
        # make sure service call was successful
        if not response.success:
            rospy.WARN("{} call to apply_joint_effort failed!".format(self.joint_name))
    