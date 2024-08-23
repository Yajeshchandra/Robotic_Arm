import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_commander import MoveGroupCommander
# from std_msgs.msg import String

# Initialize moveit_commander and rospy
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

# Instantiate a `RobotCommander`_ object. This object is an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()

# Instantiate a `PlanningSceneInterface`_ object. This object is an interface to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a `MoveGroupCommander`_ object. This object is an interface to one group of joints.
group_name = "arm"  # replace "arm" with your MoveIt group name
move_group = MoveGroupCommander(group_name)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Set the final joint values you want to reach
joint_goal = move_group.get_current_joint_values()
# Set your target joint angles (in radians)
joint_goal[0] += 0.1  # Joint 1
joint_goal[1] += 0.5  # Joint 2
joint_goal[2] += 0.5  # Joint 3
joint_goal[3] += 0.0  # Joint 4
joint_goal[4] += 0.0  # Joint 5

# Plan and execute the motion
move_group.go(joint_goal, wait=True)

# Calling `stop()` ensures that there is no residual movement
move_group.stop()

# Optionally, you can also visualize the trajectory
trajectory = move_group.plan(joint_goal)

# Display the planned trajectory
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(trajectory)
display_trajectory_publisher.publish(display_trajectory)

# When finished shut down moveit_commander.
moveit_commander.roscpp_shutdown()
