#!/usr/bin/env python

import rospy
import moveit_commander
import math

def rotate_joint5_by_deg(angle_deg=10.0):
    # 初始化 MoveIt Commander
    moveit_commander.roscpp_initialize([])
    rospy.init_node('rotate_joint5_node', anonymous=True)

    group = moveit_commander.MoveGroupCommander("cobot_arm")  # 根据你的 planning group 名称调整

    # 读取当前关节位置
    joint_values = group.get_current_joint_values()
    rospy.loginfo("当前关节角度(rad): %s", joint_values)

    # 第5轴在列表中的索引（从 0 开始计数）
    idx = 5
    angle_rad = math.radians(angle_deg)

    # 逆时针旋转：增加角度
    joint_values[idx] -= angle_rad
    group.set_joint_value_target(joint_values)

    # 拆包 plan() 返回值
    success, plan, planning_time, error_code = group.plan()
    if not success:
        rospy.logerr("规划失败，无法执行运动")
    else:
        # 只传 trajectory（plan）给 execute
        exec_success = group.execute(plan, wait=True)
        if exec_success:
            rospy.loginfo("✅ 第5关节已旋转 %.2f°", angle_deg)
        else:
            rospy.logerr("❌ 执行失败")

    group.stop()
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        rotate_joint5_by_deg(20.0)
    except rospy.ROSInterruptException:
        pass
