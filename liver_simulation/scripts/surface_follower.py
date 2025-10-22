#!/usr/bin/env python3
"""
Surface Follower Node
Generates and executes a trajectory for the robot's end-effector to follow a surface.
"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
import numpy as np
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator
import struct
import yaml
import os
from tf.transformations import euler_matrix, quaternion_from_euler

class SurfaceFollower:
    def __init__(self):
        rospy.init_node('surface_follower', anonymous=True)

        self.robot_name = rospy.get_param('~robot_name', 'iiwa')
        self.group_name = rospy.get_param('~move_group', 'manipulator')
        
        # Wait for move_group to be ready
        rospy.loginfo("Waiting for move_group to be ready...")
        rospy.sleep(8.0)  # Give move_group time to fully initialize

        # Initialize MoveIt! with the robot namespace
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander(robot_description=f'/{self.robot_name}/robot_description',
                                                      ns=f'/{self.robot_name}')
        self.scene = moveit_commander.PlanningSceneInterface(ns=f'/{self.robot_name}')
        
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name,
                                                              robot_description=f'/{self.robot_name}/robot_description',
                                                              ns=f'/{self.robot_name}')
        self.move_group.set_planning_time(10.0)
        self.move_group.set_num_planning_attempts(10)
        
        rospy.loginfo(f"Connected to move_group '{self.group_name}' in namespace '/{self.robot_name}'")
        rospy.loginfo(f"Planning frame: {self.move_group.get_planning_frame()}")
        rospy.loginfo(f"End effector link: {self.move_group.get_end_effector_link()}")

        # Load robot spawn pose to align world and robot frames
        default_scene_config = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                            'config', 'scene_objects.yaml')
        scene_config_path = rospy.get_param('~scene_config', default_scene_config)
        self.world_to_base_translation, self.world_to_base_rotation = self._load_robot_spawn_transform(scene_config_path)

        # Subscribe to the point cloud
        point_cloud_topic = '/simulated_surface/points'
        rospy.loginfo(f"Waiting for point cloud on {point_cloud_topic}...")
        self.cloud_sub = rospy.Subscriber(point_cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)

        rospy.loginfo("Surface follower initialized and ready.")

    def _load_robot_spawn_transform(self, config_path):
        """Load robot spawn pose from scene config to transform world to base frame."""
        translation = np.array([0.0, 0.0, 0.0])
        rotation = np.identity(3)
        try:
            with open(config_path, 'r') as config_file:
                config = yaml.safe_load(config_file) or {}
                spawn = config.get('robot_spawn_pose', {})
                translation = np.array([
                    float(spawn.get('x', 0.0)),
                    float(spawn.get('y', 0.0)),
                    float(spawn.get('z', 0.0)),
                ])
                roll = float(spawn.get('roll', 0.0))
                pitch = float(spawn.get('pitch', 0.0))
                yaw = float(spawn.get('yaw', 0.0))
                base_to_world = euler_matrix(roll, pitch, yaw)[:3, :3]
                rotation = base_to_world.T
        except Exception as e:
            rospy.logwarn(f"Failed to load robot spawn pose from {config_path}: {e}. Using identity transform.")
        return translation, rotation

    def parse_pointcloud2(self, cloud_msg):
        """Parse PointCloud2 message into numpy array"""
        points = []
        point_step = cloud_msg.point_step
        for i in range(cloud_msg.width):
            offset = i * point_step
            x, y, z = struct.unpack('fff', cloud_msg.data[offset:offset+12])
            points.append([x, y, z])
        return np.array(points)

    def transform_points_to_base(self, points_world):
        """Transform points from world frame into robot base frame"""
        translated = points_world - self.world_to_base_translation
        points_base = translated.dot(self.world_to_base_rotation)
        return points_base

    def cloud_callback(self, cloud_msg):
        """Once point cloud is received, generate and execute trajectory."""
        rospy.loginfo("========================================")
        rospy.loginfo("Point cloud received. Generating trajectory...")
        rospy.loginfo("========================================")
        self.cloud_sub.unregister() # We only need one snapshot

        try:
            points_world = self.parse_pointcloud2(cloud_msg)
            if len(points_world) == 0:
                rospy.logwarn("Point cloud is empty, cannot generate trajectory.")
                return

            rospy.loginfo(f"Parsed {len(points_world)} points from cloud")
            
            points_base = self.transform_points_to_base(points_world)
            rospy.loginfo(f"Transformed points to base frame")
            rospy.loginfo(f"Point cloud bounds in base frame: min={np.min(points_base, axis=0)}, max={np.max(points_base, axis=0)}")

            # Create interpolators for the surface (linear with nearest fallback)
            linear_interp = LinearNDInterpolator(points_base[:, :2], points_base[:, 2], fill_value=np.nan)
            nearest_interp = NearestNDInterpolator(points_base[:, :2], points_base[:, 2])

            def to_float(value):
                """Convert interpolator output to float, handling numpy scalars/arrays."""
                arr = np.asarray(value)
                if arr.size == 0:
                    return np.nan
                return float(arr.reshape(-1)[0])

            # Define a simple line scan path in the base frame
            # Let's scan along the x-axis of the point cloud
            min_x, max_x = np.min(points_base[:, 0]), np.max(points_base[:, 0])
            scan_y = np.mean(points_base[:, 1])
            num_steps = 20
            
            rospy.loginfo(f"Planning scan from x={min_x:.3f} to x={max_x:.3f} at y={scan_y:.3f}")
            
            waypoints = []
            tool_offset = 0.05 # Keep the tool 5cm above the surface

            for x in np.linspace(min_x, max_x, num_steps):
                # For each (x, y), find the z on the surface using linear interpolation
                z_val = to_float(linear_interp(x, scan_y))
                if np.isnan(z_val):
                    # Fall back to nearest neighbour if outside convex hull
                    z_val = to_float(nearest_interp(x, scan_y))

                if np.isnan(z_val):
                    rospy.logwarn(f"Could not interpolate z for x={x:.3f}, y={scan_y:.3f}")
                    continue

                pose = Pose()
                pose.position.x = x
                pose.position.y = scan_y
                pose.position.z = z_val + tool_offset

                # Keep the tool pointing down (perpendicular to the XY plane)
                # This is a simple orientation. For a curved surface, we'd calculate normals.
                q = quaternion_from_euler(-np.pi/2, 0, 0) # Adjust as needed for your tool
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                
                waypoints.append(copy.deepcopy(pose))

            if not waypoints:
                rospy.logerr("Could not generate any valid waypoints.")
                return

            rospy.loginfo(f"Generated {len(waypoints)} waypoints.")
            rospy.loginfo(f"First waypoint: pos=({waypoints[0].position.x:.3f}, {waypoints[0].position.y:.3f}, {waypoints[0].position.z:.3f})")
            rospy.loginfo(f"Last waypoint: pos=({waypoints[-1].position.x:.3f}, {waypoints[-1].position.y:.3f}, {waypoints[-1].position.z:.3f})")
            rospy.loginfo("Planning Cartesian path...")

            # Plan the Cartesian path
            self.move_group.set_start_state_to_current_state()
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold

            if fraction < 0.9:
                rospy.logerr(f"Could only plan {fraction*100:.2f}% of the trajectory. Aborting.")
                return

            rospy.loginfo(f"Cartesian path planned successfully ({fraction*100:.1f}% coverage). Executing...")
            
            # Execute the plan
            success = self.move_group.execute(plan, wait=True)
            self.move_group.stop()
            if not success:
                rospy.logerr("MoveIt! execution reported failure")
                return
            rospy.loginfo("Trajectory execution complete!")
            rospy.loginfo("========================================")
            
        except Exception as e:
            rospy.logerr(f"Error in surface follower: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

if __name__ == '__main__':
    try:
        follower = SurfaceFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
