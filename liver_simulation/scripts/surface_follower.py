#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import tf
import copy
from geometry_msgs.msg import Pose, Quaternion

class SurfaceFollower:
    def __init__(self):
        """
        Initializes the SurfaceFollower node, sets up MoveIt commanders for the robot and scene,
        and waits for the liver phantom to appear in the planning scene.
        """
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('surface_follower', anonymous=True)

        robot_name = rospy.get_param('~robot_name', 'iiwa')
        ns = f"/{robot_name.strip('/')}"
        self.robot = moveit_commander.RobotCommander(robot_description="robot_description", ns=ns)
        self.scene = moveit_commander.PlanningSceneInterface(ns=ns)
        
        group_name = "manipulator"  # Replace with your robot's planning group if different
        self.move_group = moveit_commander.MoveGroupCommander(group_name, robot_description=f"/{robot_name}/robot_description", ns=robot_name)
        
        self.move_group.set_planning_time(10.0) # seconds

        # Allow some time for MoveIt to initialize
        rospy.sleep(2.0)

        self.liver_object = None
        self.find_liver_phantom()

    def find_liver_phantom(self):
        """
        Waits for the 'liver_phantom' collision object to appear in the planning scene
        and retrieves its details.
        """
        rospy.loginfo("Waiting for liver_phantom to appear in the planning scene...")
        while 'liver_phantom' not in self.scene.get_known_object_names() and not rospy.is_shutdown():
            rospy.sleep(0.5)
        
        if rospy.is_shutdown():
            return

        rospy.loginfo("Found liver_phantom in the planning scene.")
        self.liver_object = self.scene.get_objects(['liver_phantom'])['liver_phantom']

    def follow_surface(self):
        """
        Plans and executes a sweeping motion over the top surface of the liver phantom.
        """
        if not self.liver_object:
            rospy.logerr("Liver phantom not found. Aborting surface following.")
            return

        # 2. Define the sweep path based on the liver's geometry
        liver_pose = self.liver_object.pose
        liver_dims = self.liver_object.primitives[0].dimensions # [length, width, height]

        # Sweep along the top surface (X-Y plane of the box)
        
        # Create a target pose with a specific, downward-facing orientation
        start_pose = Pose()
        
        # Set position to touch the liver surface (at the top surface level)
        start_pose.position.x = liver_pose.position.x
        start_pose.position.y = liver_pose.position.y
        start_pose.position.z = liver_pose.position.z + liver_dims[2] / 2.0  # At liver surface level

        # Set orientation to point straight down (180 degrees rotation around X-axis)
        q = tf.transformations.quaternion_from_euler(-3.14159, 0, 0) # Roll, Pitch, Yaw
        start_pose.orientation = Quaternion(*q)
        rospy.loginfo(f"Target starting pose: {start_pose}")

        # Move to the starting position first (at liver surface for contact)
        rospy.loginfo("Moving to starting position at liver surface for contact.")
        self.move_group.set_pose_target(start_pose)
        
        # Plan and execute, with better error checking
        success = self.move_group.go(wait=True)
        if not success:
            rospy.logerr("Failed to move to the starting position. Is the goal reachable and not in collision? Check RViz.")
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            return

        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(1)

        waypoints = []
        base_pose = self.move_group.get_current_pose().pose

        # Liver dimensions: [length, width, height] = [0.2, 0.2, 0.1]
        liver_length = liver_dims[0]  # X dimension
        liver_width = liver_dims[1]   # Y dimension
        
        # Half dimensions for corner calculations
        half_length = liver_length / 2.0
        half_width = liver_width / 2.0

        rospy.loginfo("Generating border-following waypoints...")
        
        # Start from current position (above center)
        # Create waypoints that trace around the liver's border in a rectangular pattern
        
        # Waypoint 1: Move to front-left corner of liver
        wp1 = copy.deepcopy(base_pose)
        wp1.position.x = liver_pose.position.x - half_length
        wp1.position.y = liver_pose.position.y - half_width
        waypoints.append(wp1)
        
        # Waypoint 2: Move to front-right corner
        wp2 = copy.deepcopy(wp1)
        wp2.position.x = liver_pose.position.x + half_length
        wp2.position.y = liver_pose.position.y - half_width
        waypoints.append(wp2)
        
        # Waypoint 3: Move to back-right corner
        wp3 = copy.deepcopy(wp2)
        wp3.position.x = liver_pose.position.x + half_length
        wp3.position.y = liver_pose.position.y + half_width
        waypoints.append(wp3)
        
        # Waypoint 4: Move to back-left corner
        wp4 = copy.deepcopy(wp3)
        wp4.position.x = liver_pose.position.x - half_length
        wp4.position.y = liver_pose.position.y + half_width
        waypoints.append(wp4)
        
        # Waypoint 5: Return to front-left corner to complete the rectangle
        wp5 = copy.deepcopy(wp4)
        wp5.position.x = liver_pose.position.x - half_length
        wp5.position.y = liver_pose.position.y - half_width
        waypoints.append(wp5)

        # 3. Plan and execute the trajectory
        rospy.loginfo(f"Executing sweep across {len(waypoints)} waypoints.")
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.02,        # eef_step (increased for smoother path)
                                       True)        # avoid_collisions (enabled for safety)
        
        if fraction > 0.9: # Execute if path is mostly complete
            self.move_group.execute(plan, wait=True)
            rospy.loginfo("Surface following complete.")
        else:
            rospy.logwarn(f"Could not compute a valid path for the sweep (fraction: {fraction}).")


if __name__ == '__main__':
    try:
        follower = SurfaceFollower()
        if follower.liver_object:
            follower.follow_surface()
        else:
            rospy.logerr("Could not find liver phantom, shutting down.")
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Shutting down surface follower node.")
        moveit_commander.roscpp_shutdown()
