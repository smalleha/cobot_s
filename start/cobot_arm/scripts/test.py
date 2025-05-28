#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy, sys, math
import rospy, roslib, numpy
import moveit_commander, tf, tf2_ros
from moveit_commander import RobotCommander
from moveit_commander import MoveGroupCommander
from std_srvs.srv import Trigger
from dh_gripper_msgs.msg import GripperCtrl
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
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

def move_robot():
    # 初始化 ROS 节点和 MoveIt! Commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_cartesian_example", anonymous=True)

    # 初始化 PlanningSceneInterface 和 MoveGroupCommander
    # scene = PlanningSceneInterface()
    group = MoveGroupCommander("cobot_arm")  # 控制的机械臂组，确保在这里使用正确的机械臂名称

    # 获取当前机械臂末端执行器的位姿
    current_pose = group.get_current_pose().pose
    rospy.loginfo("Current pose: %s", current_pose)

    # 定义目标位姿，目标是将末端执行器向前移动0.1米
    target_pose = current_pose
    target_pose.position.x += 0.1  # 沿x轴向前移动0.1米

    # 创建路径点列表并加入当前位姿和目标位姿
    waypoints = []
    waypoints.append(current_pose)  # 当前位姿
    waypoints.append(target_pose)   # 目标位姿

    # 设置每个步进的步长和跳跃阈值
    eef_step = 0.01  # 每步的步进值
    jump_threshold = 0.0  # 设置跳跃阈值，通常设置为0

    # 计算笛卡尔路径
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,   # 路点列表
        eef_step,    # 终端步进值
        avoid_collisions=True)  # 避免碰撞

    # 检查路径规划是否成功
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the arm.")
        rospy.sleep(1)
        group.execute(plan, wait=True)
    else:
        rospy.logwarn("Path planning failed with only {:.2f} success.".format(fraction))

    # 关闭 MoveIt! Commander
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
