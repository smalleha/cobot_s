#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy, sys, math
import rospy, roslib, numpy,time
import moveit_commander, tf, tf2_ros
from moveit_commander import RobotCommander
from moveit_commander import MoveGroupCommander
from std_srvs.srv import Trigger
from dh_gripper_msgs.msg import GripperCtrl
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Twist
from move_base_msgs.msg import MoveBaseActionResult
from copy import deepcopy
from moveit_python import MoveGroupInterface
# from moveit_msgs.msg import Constraints, JointConstraint
import moveit_python
import moveit_msgs.msg
import moveit_commander

from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from ar_track_alvar_msgs.msg import AlvarMarkers

PICK, OBJECT, PLACE = 0, 1, 2

class MoveItPlanningDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node("cobot_move_grab")
        self.temp = {PICK: Pose(), OBJECT: Pose(), PLACE: Pose()}
        self.seq_count = {PICK: 0, OBJECT: 0, PLACE: 0}
        self.real_position = {PICK: Pose(), OBJECT: Pose(), PLACE: Pose()}

        self.tf_listener = tf.TransformListener()
        self.gripper_pub = rospy.Publisher("/gripper/ctrl", GripperCtrl, queue_size=1, latch=True)
        self.nav_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True) 
        self.del_traj_srv = rospy.ServiceProxy("/rm_driver/Del_All_Traj", Trigger)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
        self.grab_param = rospy.get_param('/move_grab/grab_param', default='grab_pose')

        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.NavIsReachGoal)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_callback)

        # 获取示教的导航点
        self.nav_goal_lists = []
        if not self.get_nav_goals():
            rospy.logwarn("get_nav_goals failed!")
            sys.exit()
        self.nav_status = -1

        # 初始化需要使用move group控制的机械臂中的self.arm group
        self.arm = moveit_commander.MoveGroupCommander("cobot_arm")
        # 创建一个PlanningSceneInterface对象，用于添加物体到场景中
        self.scene = moveit_commander.PlanningSceneInterface()
        # 设置目标位置所使用的参考坐标系
        # self.reference_frame = "arm_base"
        # self.arm.set_pose_reference_frame(self.reference_frame)

        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_max_acceleration_scaling_factor(0.3)

        # 当运动规划失败后，允许重新规划
        # self.arm.allow_replanning(True)
        # 设置每次运动规划的时间限制：5s
        # self.arm.set_planning_time(5)

        self.id_num = int()
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        # 打开夹爪
        self.gripper_open()
        self.tf_flag = True
        self.flag = False
        self.gripper_flag = True

        self.gripper_z = 0.16
        self.gripper_x = 0.1
        self.gripper_y = 0.0
        # 控制机械臂先回到home位置
        # self.move2home()

    def gripper_open(self):
        gripper_ctl = GripperCtrl()
        gripper_ctl.position = 1000.0
        self.gripper_pub.publish(gripper_ctl)
        print ("gripper_open")

    def gripper_close(self):
        gripper_ctl = GripperCtrl()
        gripper_ctl.position = 0.0
        self.gripper_pub.publish(gripper_ctl)
        print ("gripper_close")

    def move2home(self):
        self.arm.set_named_target("home")
        success = False
        attempts = 0
        max_attempts = 2  # 最多尝试两次
        # res = self.del_traj_srv.call()
        # if not res.success:
        #     rospy.logwarn(res.message)
        while not success and attempts < max_attempts:
            success = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
        
            if not success:
                attempts += 1
                print(f"Move to initial pose failed. Retrying... (Attempt {attempts}/{max_attempts})")


        print("initial")
        rospy.sleep(1)

    def move2initial(self):
        self.arm.set_named_target("initial")
    
        success = False
        attempts = 0
        max_attempts = 2  # 最多尝试两次
        # res = self.del_traj_srv.call()
        # if not res.success:
        #     rospy.logwarn(res.message)
        while not success and attempts < max_attempts:
            success = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
        
            if not success:
                attempts += 1
                print(f"Move to initial pose failed. Retrying... (Attempt {attempts}/{max_attempts})")


        print("initial")
        rospy.sleep(1)

    def move2back(self):
        self.arm.set_named_target("back")
    
        success = False
        attempts = 0
        max_attempts = 2  # 最多尝试两次
        # res = self.del_traj_srv.call()
        # if not res.success:
        #     rospy.logwarn(res.message)
        while not success and attempts < max_attempts:
            success = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
        
            if not success:
                attempts += 1
                print(f"Move to back pose failed. Retrying... (Attempt {attempts}/{max_attempts})")


        print("back")
        rospy.sleep(3)

    def marker_callback(self, msg):

        if msg.markers:
            self.get_pickup_point(msg)

    def get_pickup_point(self, msg):
        for marker in msg.markers:
            self.id_num = marker.id
            if self.id_num in (PICK, OBJECT, PLACE):

                self.temp[self.id_num].position.x += marker.pose.pose.position.x
                self.temp[self.id_num].position.y += marker.pose.pose.position.y
                self.temp[self.id_num].position.z += marker.pose.pose.position.z

                count = self.seq_count[self.id_num]
                self.seq_count[self.id_num] += 1

                if count == 2:
                    self.real_position[self.id_num].position.x = self.temp[self.id_num].position.x / 3.0
                    self.real_position[self.id_num].position.y = self.temp[self.id_num].position.y / 3.0
                    self.real_position[self.id_num].position.z = self.temp[self.id_num].position.z / 3.0
                    self.temp[self.id_num] = Pose()
                    self.seq_count[self.id_num] = 0

    def NavIsReachGoal(self, msg):
        self.nav_status = msg.status.status
        if msg.status.status == 3:
            print("Reach goal!")
        if msg.status.status == 4:
            print("Abort goal!")
    #从配置文件中读取目标点参数值
    def get_nav_goals(self):
        goal_num = rospy.get_param("~nav_goal_num", 0)
        print("nav goal num: %d" % goal_num)
        for n in range(1, goal_num + 1):
            goal_id = "nav_goal_" + str(n)
            pose = rospy.get_param("~" + goal_id)
            print(goal_id, pose)
            nav_goal = PoseStamped()
            nav_goal.header.frame_id = 'map'
            nav_goal.header.stamp = rospy.get_rostime()
            nav_goal.pose.position.x = pose['x']
            nav_goal.pose.position.y = pose['y']
            nav_goal.pose.position.z = pose['z']
            nav_goal.pose.orientation.x = pose['qx']
            nav_goal.pose.orientation.y = pose['qy']
            nav_goal.pose.orientation.z = pose['qz']
            nav_goal.pose.orientation.w = pose['qw']
            # print(nav_goal)
            #将读取到的参数值存放在nav_goal_lists列表中
            self.nav_goal_lists.append(nav_goal)
        if goal_num > 0:
            return True
        else:
            return False

    def move_waypoints(self, target_pose):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        try:
            transform = tf_buffer.lookup_transform("arm_base", self.end_effector_link, rospy.Time(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to lookup transform from arm_base to end_effector_link")
        start_current_pose = self.arm.get_current_pose().pose

        start_pose = Pose()
        point1 = Pose()
        point2 = Pose()
        point3 = Pose()

        start_pose = start_current_pose

        waypoints = []
        waypoints.append(start_pose)

        # 移动Y轴
        point2.position.x = start_pose.position.x
        point2.position.y = target_pose.position.y
        point2.position.z = start_pose.position.z
        point2.orientation = start_pose.orientation
        waypoints.append(copy.deepcopy(point2))

        # 移动Z轴
        point3.position.x = start_pose.position.x
        point3.position.y = target_pose.position.y
        point3.position.z = target_pose.position.z
        point3.orientation = start_pose.orientation
        waypoints.append(copy.deepcopy(point3))

        # 移动X轴
        point1.position.x = target_pose.position.x
        point1.position.y = target_pose.position.y
        point1.position.z = target_pose.position.z
        point1.orientation = start_pose.orientation
        waypoints.append(copy.deepcopy(point1))

        # 创建一个Box对象，描述盒子的尺寸
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"  # 盒子在哪个坐标系下
        box_pose.pose.orientation.w = 1.0  # 盒子的姿态
        box_pose.pose.position.x = 0.5  # 盒子的位置
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.7
        box_size = (0.2, 0.2, 0.05)  # 盒子的尺寸

        # 在场景中添加一个盒子
        self.scene.add_box("box", box_pose, box_size)
        # 创建一个Box对象，描述盒子的尺寸
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = "base_link"  # 盒子在哪个坐标系下
        box2_pose.pose.orientation.w = 1.0  # 盒子的姿态
        box2_pose.pose.position.x = 0.5  # 盒子的位置
        box2_pose.pose.position.y = 0.0
        box2_pose.pose.position.z = 1.1
        box2_size = (0.2, 0.2, 0.05)  # 盒子的尺寸

        # 在场景中添加一个盒子
        self.scene.add_box("box2", box2_pose, box2_size)

        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,
                                    avoid_collisions=True) # avoid_collisions，避障规划            
            # 尝试次数累加
            attempts += 1
            
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(1)        
        self.scene.remove_world_object("box")
        self.scene.remove_world_object("box2")
        self.target_pose = Pose()

    def move_joint5(self,angle_deg):
        joint_values = self.arm.get_current_joint_values()

        # 第5轴在列表中的索引（从 0 开始计数）
        idx = 5
        angle_rad = math.radians(angle_deg)

        # 逆时针旋转：增加角度
        joint_values[idx] -= angle_rad
        self.arm.set_joint_value_target(joint_values)

        # 拆包 plan() 返回值
        success, plan, planning_time, error_code = self.arm.plan()
        if not success:
            rospy.logerr("规划失败，无法执行运动")
        else:
            # 只传 trajectory（plan）给 execute
            exec_success = self.arm.execute(plan, wait=True)
            if exec_success:
                rospy.loginfo("第5关节已旋转 %.2f°", angle_deg)
            else:
                rospy.logerr("执行失败")


    def move_straight_pose(self, scale=1):
        rospy.sleep(2)

        waypoints = []
        current_pose = self.arm.get_current_pose().pose
        start_pose = Pose()
        start_pose = current_pose

        wpose = start_pose
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z += scale * 0.0
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints, 0.01, 0.0, avoid_collisions=True # waypoints to follow  # eef_step
        )  # jump_threshold

        if(fraction < 0.9):
            rospy.logwarn("compute_cartesian_path failed! fraction = %f", fraction)
        else:
            self.arm.execute(plan, wait=True)
            rospy.sleep(1)

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return fraction

    def get_tf(self, parent_frame, child_frame):
        try:
            self.target_pose = Pose()

            current_pose = self.arm.get_current_pose().pose
            if child_frame == "grab_pose" and self.id_num == 0:
                self.target_pose.position.x = current_pose.position.x + self.real_position[0].position.z - self.gripper_z
                self.target_pose.position.y = current_pose.position.y + (-self.real_position[0].position.y) + self.gripper_y
                self.target_pose.position.z = current_pose.position.z + (-self.real_position[0].position.x) +self.gripper_x

            elif child_frame == "place_pose" and self.id_num == 2:
                self.target_pose.position.x = current_pose.position.x + self.real_position[2].position.z - self.gripper_z
                self.target_pose.position.y = current_pose.position.y + (-self.real_position[2].position.y) + self.gripper_y
                self.target_pose.position.z = current_pose.position.z + (-self.real_position[2].position.x) +self.gripper_x
            else :
                return False

            rospy.loginfo(f"Current pose: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
            rospy.loginfo(f"real_position: x={self.real_position[self.id_num].position.x:.3f},y={self.real_position[self.id_num].position.y:.3f}, z={self.real_position[self.id_num].position.z:.3f}")

            rospy.loginfo(self.target_pose)

            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
            rospy.logwarn(e)
            return False

    def calibration(self,target_pose):
        del_x = target_pose.position.x - 0.9
        ticks = 0
        rate = 10
        loop_rate = rospy.Rate(rate)
        linear_speed = 0.0
        linear_duration = 0.0
        speed =Twist()
        if del_x > 0:
            linear_speed = 0.2
        else:
            linear_speed = -0.2     
        linear_duration = abs(del_x/linear_speed)
        speed.linear.x = linear_speed
        ticks = int(linear_duration * rate)
        rospy.logwarn("ticks: %d",ticks)
        rospy.logwarn("linear_duration: %f",linear_duration)
        rospy.logwarn("del_x: %f",del_x)
        
        for i in range(ticks):
            self.vel_pub.publish(speed)
            loop_rate.sleep()
        speed.linear.x = 0
        self.vel_pub.publish(speed)
        sleep_duration = 2 + math.ceil(linear_duration)
        time.sleep(sleep_duration)


    def run(self):
        start_goal = True
        for nav_goal in self.nav_goal_lists:
            self.nav_goal_pub.publish(nav_goal)
            #3代表到达目标点；4代表未到达目标点
            while self.nav_status != 3 and self.nav_status != 4:
                rospy.sleep(1.0)
            if self.nav_status == 4:
                self.nav_status = -1
                continue
            self.nav_status = -1

            if start_goal:
                rospy.loginfo("reach start position")
                start_goal = False
                continue

            self.move2initial()
            print("11111")
            if(self.gripper_flag):
                print("grab_pose_222")
                if(self.get_tf("arm_base", "grab_pose")):
                    print("grab_pose")
                    # self.add_box_to_scene()
                    if self.target_pose.position.x > 1.1:
                        rospy.logwarn("Target_pose.position.x is too large")
                        rospy.logwarn("target_pose.position.x: %f",self.target_pose.position.x)
                        self.calibration(self.target_pose)
                        self.get_tf("arm_base", "grab_pose")
                        self.move_waypoints(self.target_pose)
                    
                    self.move_waypoints(self.target_pose)
                    self.gripper_close()
                    rospy.sleep(1)
                    self.move_joint5(20)

                    self.move_straight_pose()
                    self.move2initial()
                    self.move2home()
                    self.gripper_flag = False
                print("2222")
            else:
                print("3333")
                if(self.get_tf("arm_base", "place_pose")):
                    print("place_pose")
                    if self.target_pose.position.x > 1.1:
                        rospy.logwarn("Target_pose.position.x is too large")
                        rospy.logwarn("target_pose.position.x: %f",self.target_pose.position.x)
                        self.calibration(self.target_pose)
                        self.get_tf("arm_base", "place_pose")
                        self.move_waypoints(self.target_pose)
                    
                    self.move_waypoints(self.target_pose)
                    self.gripper_open()
                    rospy.sleep(1)

                    self.move_straight_pose()
                    self.move2initial()

                rospy.sleep(1)
            
        self.nav_goal_pub.publish(self.nav_goal_lists[0])
        while self.nav_status != 3 and self.nav_status != 4:
            rospy.sleep(1.0)
        rospy.loginfo("reback start position, exit")

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    node = MoveItPlanningDemo()
    node.run()
