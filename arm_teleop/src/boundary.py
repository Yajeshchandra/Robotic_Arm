#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from random import uniform
import sys

def generate_workspace_boundary(group, samples=1000):
    marker_pub = rospy.Publisher('workspace_boundary', Marker, queue_size=10)
    marker = Marker()
    marker.header.frame_id = group.get_planning_frame()
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.scale.x = 0.01  # size of the points
    marker.scale.y = 0.01
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    joint_limits = {
        "joint_0": (-3.14, 3.14),  # Replace with actual joint limits
        "joint_1": (-1.57, 1.57),
        "joint_2": (-3.14, 3.14),
        "joint_3": (-9, 9),
        "joint_4": (-3.14, 0)
    }

    joint_names = group.get_active_joints()

    for _ in range(samples):
        joint_values = [uniform(joint_limits[joint][0], joint_limits[joint][1]) for joint in joint_names]
        group.set_joint_value_target(joint_values)
        plan = group.plan()
        if not plan[0]:
            continue

        pose = group.get_current_pose().pose
        point = geometry_msgs.msg.Point()
        point.x = pose.position.x
        point.y = pose.position.y
        point.z = pose.position.z
        marker.points.append(point)

    marker_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node('workspace_boundary_generator')

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"  # Replace with your MoveIt group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    generate_workspace_boundary(move_group, samples=10000)

    rospy.spin()
