#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import tf
import copy
import math
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
        
        # Set end effector to probe tip
        self.move_group.set_end_effector_link("probe_link_ee")
        
        # Get probe length from parameter server
        self.probe_length = rospy.get_param(f"{ns}/probe_length", 0.24)
        self.surface_clearance = rospy.get_param('~surface_clearance', 0.01)
        rospy.loginfo(f"Using probe length: {self.probe_length}, surface clearance: {self.surface_clearance}")
        
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
        
        rospy.loginfo("Moving to home position.")
        self.move_group.set_named_target("home")
        
        # Plan and execute
        success = self.move_group.go(wait=True)
        if not success:
            rospy.logerr("Failed to move to home position.")
            self.move_group.stop()
            return

        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(1)

        waypoints = []

        # Liver dimensions: [length, width, height] = [0.2, 0.2, 0.1]
        liver_length = liver_dims[0]  # X dimension
        liver_width = liver_dims[1]   # Y dimension
        
        # Half dimensions for corner calculations
        half_length = liver_length / 2.0
        half_width = liver_width / 2.0

        # First, move probe tip to be just above the liver surface center
        rospy.loginfo("Moving probe to liver surface center with probe pointing down...")
        liver_top_z = liver_pose.position.z + (liver_dims[2] / 2.0)  # Top of liver
        approach_pose = Pose()
        approach_pose.position.x = liver_pose.position.x
        approach_pose.position.y = liver_pose.position.y
        approach_pose.position.z = liver_top_z + self.surface_clearance + 0.3  # Start higher for safety
        
        # Orient the probe to point straight down (end effector Z-axis pointing down)
        # Quaternion for 180 degree rotation around Y-axis (probe points down in -Z direction)
        approach_pose.orientation.x = 0.0
        approach_pose.orientation.y = 1.0
        approach_pose.orientation.z = 0.0
        approach_pose.orientation.w = 0.0
        
        self.move_group.set_pose_target(approach_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if not success:
            rospy.logerr("Failed to move probe to liver surface. Aborting.")
            return
        
        rospy.loginfo("Successfully positioned probe above liver!")
        rospy.sleep(1)
        
        # Now move down to touch the liver surface
        rospy.loginfo("Moving probe down to touch liver surface...")
        touch_pose = Pose()
        touch_pose.position.x = liver_pose.position.x
        touch_pose.position.y = liver_pose.position.y
        touch_pose.position.z = liver_top_z + self.surface_clearance
        touch_pose.orientation = approach_pose.orientation
        
        self.move_group.set_pose_target(touch_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if not success:
            rospy.logerr("Failed to touch liver surface. Aborting.")
            return
        
        rospy.loginfo("Successfully touched liver surface!")
        rospy.sleep(1)

        # Execute sweep motion
        rospy.loginfo("Generating border-following waypoints...")
        
        # Now create waypoints that trace around the liver's border in a rectangular pattern
        # All waypoints will be at the same Z height (liver surface + clearance)
        target_z = liver_top_z + self.surface_clearance
        
        # Waypoint 1: Move to front-left corner of liver
        wp1 = Pose()
        wp1.position.x = liver_pose.position.x - half_length
        wp1.position.y = liver_pose.position.y - half_width
        wp1.position.z = target_z
        wp1.orientation = approach_pose.orientation
        waypoints.append(wp1)
        
        # Waypoint 2: Move to front-right corner
        wp2 = Pose()
        wp2.position.x = liver_pose.position.x + half_length
        wp2.position.y = liver_pose.position.y - half_width
        wp2.position.z = target_z
        wp2.orientation = approach_pose.orientation
        waypoints.append(wp2)
        
        # Waypoint 3: Move to back-right corner
        wp3 = Pose()
        wp3.position.x = liver_pose.position.x + half_length
        wp3.position.y = liver_pose.position.y + half_width
        wp3.position.z = target_z
        wp3.orientation = approach_pose.orientation
        waypoints.append(wp3)
        
        # Waypoint 4: Move to back-left corner
        wp4 = Pose()
        wp4.position.x = liver_pose.position.x - half_length
        wp4.position.y = liver_pose.position.y + half_width
        wp4.position.z = target_z
        wp4.orientation = approach_pose.orientation
        waypoints.append(wp4)
        
        # Waypoint 5: Return to front-left corner to complete the rectangle
        wp5 = Pose()
        wp5.position.x = liver_pose.position.x - half_length
        wp5.position.y = liver_pose.position.y - half_width
        wp5.position.z = target_z
        wp5.orientation = approach_pose.orientation
        waypoints.append(wp5)

        # Plan and execute the trajectory
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
