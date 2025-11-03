import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import os
import sys
import tf
import copy
import math
from geometry_msgs.msg import Pose, Quaternion
from gazebo_msgs.msg import ContactsState

# ANSI color codes for terminal output
GREEN = '\033[92m'
RESET = '\033[0m'

def log_success(msg):
    """Log success messages in green"""
    rospy.loginfo(f"{GREEN}{msg}{RESET}")

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
        self.surface_clearance = rospy.get_param('~surface_clearance', 0.002)
        self.contact_push = rospy.get_param('~contact_push', 0.002)
        rospy.loginfo(
            f"Using probe length: {self.probe_length}, surface clearance: {self.surface_clearance}, contact push: {self.contact_push}"
        )
        
        self.move_group.set_planning_time(10.0) # seconds
        
        self.contact_data = None
        self.contact_sub = rospy.Subscriber(
            f"/{robot_name}/ft_sensor/wrench",
            ContactsState,
            self._contact_callback
        )

        # Allow some time for MoveIt to initialize
        rospy.sleep(2.0)

        self.liver_object = None
        self.find_liver_phantom()
    
    def _contact_callback(self, msg):
        """Callback for contact sensor data."""
        self.contact_data = msg

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
        
        rospy.loginfo(f"DEBUG: Liver pose: x={liver_pose.position.x:.3f}, y={liver_pose.position.y:.3f}, z={liver_pose.position.z:.3f}")
        rospy.loginfo(f"DEBUG: Liver dimensions: length={liver_dims[0]:.3f}, width={liver_dims[1]:.3f}, height={liver_dims[2]:.3f}")
        rospy.loginfo(f"DEBUG: Liver top Z: {liver_top_z:.3f}")
        rospy.loginfo(f"DEBUG: Surface clearance: {self.surface_clearance:.4f}")
        
        approach_pose = Pose()
        approach_pose.position.x = liver_pose.position.x
        approach_pose.position.y = liver_pose.position.y
        approach_pose.position.z = liver_top_z + self.surface_clearance + 0.3  # Start higher for safety (no probe_length offset needed!)
        
        rospy.loginfo(f"DEBUG: Approach pose target: x={approach_pose.position.x:.3f}, y={approach_pose.position.y:.3f}, z={approach_pose.position.z:.3f}")
        
        # Orient the probe to point straight down (end effector Z-axis pointing down)
        # Quaternion for 180 degree rotation around Y-axis (probe points down in -Z direction)
        approach_pose.orientation.x = 0.0
        approach_pose.orientation.y = 1.0
        approach_pose.orientation.z = 0.0
        approach_pose.orientation.w = 0.0
        
        rospy.loginfo(f"DEBUG: Approach orientation: x={approach_pose.orientation.x}, y={approach_pose.orientation.y}, z={approach_pose.orientation.z}, w={approach_pose.orientation.w}")
        rospy.loginfo("DEBUG: Checking if approach pose is in collision...")
        current_pose = self.move_group.get_current_pose().pose
        rospy.loginfo(f"DEBUG: Current EE pose: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
        rospy.loginfo("DEBUG: Known collision objects: " + str(self.scene.get_known_object_names()))
        ee_link = self.move_group.get_end_effector_link()
        rospy.loginfo(f"DEBUG: End effector link: {ee_link}")
        planning_frame = self.move_group.get_planning_frame()
        rospy.loginfo(f"DEBUG: Planning frame: {planning_frame}")
        
        self.move_group.set_pose_target(approach_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if not success:
            rospy.logerr("Failed to move probe to liver surface. Aborting.")
            rospy.logerr("DEBUG: Planning failed - likely due to collision or unreachable pose")
            return
        
        log_success("Successfully positioned probe above liver!")
        rospy.sleep(1)
        
        # Now move down to touch the liver surface using Cartesian path
        # We use Cartesian path with collision checking disabled for the final approach
        # because the clearance is too small for MoveIt's collision checker
        rospy.loginfo("Moving probe down to touch liver surface...")
        touch_pose = Pose()
        touch_pose.position.x = liver_pose.position.x
        touch_pose.position.y = liver_pose.position.y
        touch_pose.position.z = liver_top_z + self.surface_clearance  # probe_link_ee is at tip, no offset needed!
        touch_pose.orientation = approach_pose.orientation
        
        rospy.loginfo(f"DEBUG: Touch pose target: x={touch_pose.position.x:.3f}, y={touch_pose.position.y:.3f}, z={touch_pose.position.z:.3f}")
        
        # Use Cartesian path to move straight down with collisions disabled for final approach
        # We disable collision checking because the clearance is very small (2mm)
        waypoints_down = [touch_pose]
        
        # Slow down for careful approach
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints_down,  # waypoints to follow
            0.01,            # eef_step (1cm resolution)
            False            # avoid_collisions = False to allow close approach
        )
        
        if fraction < 0.9:  # Accept if we get at least 90% of the path
            rospy.logerr(f"Failed to compute Cartesian path to touch pose (fraction: {fraction:.2f}). Aborting.")
            # Restore normal speed
            self.move_group.set_max_velocity_scaling_factor(1.0)
            self.move_group.set_max_acceleration_scaling_factor(1.0)
            return
        
        # Retime the trajectory to ensure it's feasible
        plan = self.move_group.retime_trajectory(
            self.robot.get_current_state(),
            plan,
            velocity_scaling_factor=0.1,
            acceleration_scaling_factor=0.1
        )
        
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        # Restore normal speed
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_max_acceleration_scaling_factor(1.0)
        
        if not success:
            rospy.logerr("Failed to touch liver surface. Aborting.")
            return
        
        log_success("Successfully touched liver surface!")
        rospy.sleep(0.5)
        
        self._apply_contact_push(touch_pose)
        
        rospy.sleep(0.5)

        # Execute sweep motion
        rospy.loginfo("Generating border-following waypoints...")
        
        # Now create waypoints that trace around the liver's border in a rectangular pattern
        # All waypoints will be at the same Z height (liver surface + clearance)
        target_z = liver_top_z + self.surface_clearance  # probe_link_ee is at tip already!
        
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
        
        # Disable collision checking since we're moving along the liver surface
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step (1cm resolution)
                                       False)       # avoid_collisions = False for surface contact
        
        if fraction > 0.9: # Execute if path is mostly complete
            self.move_group.execute(plan, wait=True)
            log_success("Surface following complete.")
        else:
            rospy.logwarn(f"Could not compute a valid path for the sweep (fraction: {fraction}).")

    def _apply_contact_push(self, touch_pose):
        """
        Pushes the probe slightly into the liver surface (with collisions disabled) so
        the user can observe contact torques from the force/torque sensor.
        """
        if self.contact_push <= 0.0:
            rospy.loginfo("Contact push disabled; skipping torque measurement bump.")
            return

        # Get current pose instead of using touch_pose to ensure we start from actual position
        current_ee_pose = self.move_group.get_current_pose().pose
        contact_pose = copy.deepcopy(current_ee_pose)
        contact_pose.position.z -= self.contact_push

        rospy.loginfo(
            f"Applying contact push of {self.contact_push:.4f} m to record torque feedback."
        )
        rospy.loginfo(f"DEBUG: Current Z: {current_ee_pose.position.z:.4f}, Target Z: {contact_pose.position.z:.4f}")

        # Slow down the push for stability
        self.move_group.set_max_velocity_scaling_factor(0.05)
        self.move_group.set_max_acceleration_scaling_factor(0.05)

        plan, fraction = self.move_group.compute_cartesian_path(
            [contact_pose],
            0.002,     # eef_step (2mm resolution)
            False      # avoid_collisions = False for contact motion
        )

        if fraction < 0.9:
            rospy.logwarn(
                f"Contact push cartesian path incomplete (fraction={fraction:.2f}); skipping execution."
            )
            # Restore default scaling
            self.move_group.set_max_velocity_scaling_factor(1.0)
            self.move_group.set_max_acceleration_scaling_factor(1.0)
            return

        self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        # Wait a bit for contact to stabilize and read contact sensor
        rospy.sleep(0.5)
        
        if self.contact_data and len(self.contact_data.states) > 0:
            rospy.loginfo("=== CONTACT DETECTED ===")
            for i, contact in enumerate(self.contact_data.states):
                rospy.loginfo(f"Contact {i+1}:")
                rospy.loginfo(f"  Collision 1: {contact.collision1_name}")
                rospy.loginfo(f"  Collision 2: {contact.collision2_name}")
                
                # Sum up all contact forces
                total_force = [0.0, 0.0, 0.0]
                for wrench in contact.wrenches:
                    total_force[0] += wrench.force.x
                    total_force[1] += wrench.force.y
                    total_force[2] += wrench.force.z
                
                rospy.loginfo(f"  Total Force: x={total_force[0]:.4f}, y={total_force[1]:.4f}, z={total_force[2]:.4f} N")
                force_magnitude = math.sqrt(sum(f**2 for f in total_force))
                rospy.loginfo(f"  Force Magnitude: {force_magnitude:.4f} N")
        else:
            rospy.logwarn("No contact detected by contact sensor.")
        
        # Restore default scaling
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_max_acceleration_scaling_factor(1.0)
        
        log_success("Contact push complete. Check contact sensor readings above.")



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
