#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Move a robotic arm using Cartesian path planning in three steps:
1. Y direction
2. Z direction
3. X direction
Then return to named pose 'initial'
"""

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def move_cartesian_and_return():
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_cartesian_offset_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = rospy.get_param('~group', 'cobot_arm')
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # 获取当前姿态
    current_pose = move_group.get_current_pose().pose
    rospy.loginfo("Start pose:\n"
                  f"Position: x={current_pose.position.x:.3f}, "
                  f"y={current_pose.position.y:.3f}, "
                  f"z={current_pose.position.z:.3f}")

    # 设置偏移
    dy = 0.05
    dz = 0.05
    dx = 0.05

    waypoints = []

    # Y方向
    pose_y = Pose()
    pose_y.position.x = current_pose.position.x
    pose_y.position.y = current_pose.position.y + dy
    pose_y.position.z = current_pose.position.z
    pose_y.orientation = current_pose.orientation
    waypoints.append(pose_y)

    # Z方向
    pose_z = Pose()
    pose_z.position.x = pose_y.position.x
    pose_z.position.y = pose_y.position.y
    pose_z.position.z = pose_y.position.z + dz
    pose_z.orientation = current_pose.orientation
    waypoints.append(pose_z)

    # X方向
    pose_x = Pose()
    pose_x.position.x = pose_z.position.x + dx
    pose_x.position.y = pose_z.position.y
    pose_x.position.z = pose_z.position.z
    pose_x.orientation = current_pose.orientation
    waypoints.append(pose_x)

    # 计算路径
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        0.01,  # 轨迹插值分辨率
        0.0,   # 关节跳跃限制
        avoid_collisions=True
    )

    # 执行路径
    if fraction > 0.99:
        rospy.loginfo("执行笛卡尔路径...")
        move_group.execute(plan, wait=True)
        rospy.loginfo("路径执行完毕")
    else:
        rospy.logwarn(f"路径规划未完全成功 (fraction={fraction:.2f})，取消执行")
        return

    # 清除路径目标
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.sleep(1.0)

    # 回到初始命名位置
    rospy.loginfo("返回初始姿态 'initial'")
    move_group.set_named_target("initial")
    move_group.go(wait=True)

    # 清理
    move_group.stop()
    move_group.clear_pose_targets()
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        move_cartesian_and_return()
    except rospy.ROSInterruptException:
        pass
