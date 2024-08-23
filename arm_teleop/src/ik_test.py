#!/usr/bin/env python3

import rospy
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import PoseStamped
import tf

class IKSolver:
    def __init__(self):
        rospy.init_node('ik_solver')

        # Initialize IK solver
        self.ik_solver = IK("base_link", "link_4")

        # Set up listener for the current end-effector pose
        self.listener = tf.TransformListener()

    def get_end_effector_pose(self):
        try:
            self.listener.waitForTransform('/base_link', '/link_4', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.listener.lookupTransform('/base_link', '/link_4', rospy.Time(0))
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            return pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF Exception")
            return None

    def compute_ik(self, target_pose):
        # Extract position and orientation from the target_pose
        x = target_pose.pose.position.x
        y = target_pose.pose.position.y+0.0  # Add an offset to the y-coordinate
        z = target_pose.pose.position.z+0.01  # Add an offset to the z-coordinate

        qx = target_pose.pose.orientation.x
        qy = target_pose.pose.orientation.y
        qz = target_pose.pose.orientation.z
        qw = target_pose.pose.orientation.w

        # Use the IK solver to find the joint configuration
        seed_state = [0.0] * self.ik_solver.number_of_joints  # Use zero as the initial seed state
        joint_angles = self.ik_solver.get_ik(seed_state, x, y, z, qx, qy, qz, qw)

        if joint_angles:
            rospy.loginfo("IK Solution Found: %s", joint_angles)
        else:
            rospy.logwarn("No IK Solution Found")

        return joint_angles

    def run(self):
        # Get the current end-effector pose
        current_pose = self.get_end_effector_pose()

        if current_pose:
            # Compute the IK for the current end-effector pose
            joint_angles = self.compute_ik(current_pose)

            # Print or publish the joint angles as needed
            if joint_angles:
                rospy.loginfo("Computed joint angles: %s", joint_angles)

if __name__ == "__main__":
    try:
        ik_solver = IKSolver()
        ik_solver.run()
    except rospy.ROSInterruptException:
        pass
