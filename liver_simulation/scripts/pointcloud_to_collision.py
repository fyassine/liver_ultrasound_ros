#!/usr/bin/env python3
"""
Point Cloud to Collision Mesh Node
Subscribes to a point cloud topic and converts it to a collision mesh in MoveIt!
"""

import os
import struct
import yaml

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from sensor_msgs.msg import PointCloud2
from shape_msgs.msg import Mesh, MeshTriangle
from tf.transformations import euler_matrix


class PointCloudToCollision:
    def __init__(self):
        rospy.init_node('pointcloud_to_collision', anonymous=True)
        
        # Parameters
        self.robot_name = rospy.get_param('~robot_name', 'iiwa')
        point_cloud_topic = rospy.get_param('~point_cloud_topic', '/simulated_surface/points')
        self.mesh_name = rospy.get_param('~mesh_name', 'abdomen_surface')
        self.base_frame = rospy.get_param('~base_frame', f'{self.robot_name}_link_0')
        
        # Load robot spawn pose to align world and robot frames
        default_scene_config = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                            'config', 'scene_objects.yaml')
        scene_config_path = rospy.get_param('~scene_config', default_scene_config)
        self.world_to_base_translation, self.world_to_base_rotation = self._load_robot_spawn_transform(scene_config_path)
        
        # Publisher for planning scene
        planning_scene_topic = f'/{self.robot_name}/planning_scene'
        self.scene_pub = rospy.Publisher(planning_scene_topic, PlanningScene, queue_size=10, latch=True)
        
        # Wait a bit for MoveIt! to start
        rospy.sleep(2.0)
        
        # Subscribe to point cloud (only process once since it's static)
        rospy.loginfo(f"Waiting for point cloud on {point_cloud_topic}...")
        self.cloud_sub = rospy.Subscriber(point_cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)
        
        rospy.loginfo("Point cloud to collision converter initialized")
    
    def cloud_callback(self, cloud_msg):
        """Process point cloud and add to planning scene"""
        rospy.loginfo(f"Received point cloud with {cloud_msg.width} points in frame '{cloud_msg.header.frame_id}'")
        
        # Parse point cloud and transform into robot base frame
        points_world = self.parse_pointcloud2(cloud_msg)
        if len(points_world) == 0:
            rospy.logwarn("No points in point cloud!")
            return
        points = self.transform_points_to_base(points_world)
        
        # Log point cloud bounds for debugging
        min_pt = np.min(points, axis=0)
        max_pt = np.max(points, axis=0)
        rospy.loginfo(f"Point cloud bounds: min={min_pt}, max={max_pt}")
        
        # Create mesh from points
        mesh = self.create_mesh_from_points(points)
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.id = self.mesh_name
        collision_object.meshes.append(mesh)
        collision_object.mesh_poses.append(self.create_identity_pose())
        collision_object.operation = CollisionObject.ADD
        
        rospy.loginfo(f"Adding mesh '{self.mesh_name}' in frame '{self.base_frame}' to planning scene")
        
        # Add to planning scene
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)
        
        self.scene_pub.publish(planning_scene)
        rospy.loginfo(f"Added surface mesh '{self.mesh_name}' to planning scene with {len(mesh.triangles)} triangles")
        
        # Unsubscribe after processing (static surface)
        self.cloud_sub.unregister()
    
    def parse_pointcloud2(self, cloud_msg):
        """Parse PointCloud2 message into numpy array"""
        points = []
        point_step = cloud_msg.point_step
        
        for i in range(cloud_msg.width):
            offset = i * point_step
            x = struct.unpack('f', cloud_msg.data[offset:offset+4])[0]
            y = struct.unpack('f', cloud_msg.data[offset+4:offset+8])[0]
            z = struct.unpack('f', cloud_msg.data[offset+8:offset+12])[0]
            points.append([x, y, z])
        
        return np.array(points)

    def transform_points_to_base(self, points_world):
        """Transform points from world frame into robot base frame"""
        # Apply translation then rotation (world to base)
        translated = points_world - self.world_to_base_translation
        points_base = translated.dot(self.world_to_base_rotation)
        return points_base
    
    def create_mesh_from_points(self, points):
        """Create a solid mesh with volume from a grid of points"""
        mesh = Mesh()
        
        # Add thickness to the surface (extrude downward)
        thickness = 0.05  # 5cm thickness
        
        from scipy.spatial import Delaunay
        
        # Project points to 2D for triangulation (use X-Y coordinates)
        points_2d = points[:, :2]
        
        try:
            tri = Delaunay(points_2d)
            
            # Create top and bottom vertices
            top_vertices = []
            bottom_vertices = []
            
            for point in points:
                # Top surface vertex
                top_vertex = Point()
                top_vertex.x = float(point[0])
                top_vertex.y = float(point[1])
                top_vertex.z = float(point[2])
                top_vertices.append(top_vertex)
                mesh.vertices.append(top_vertex)
                
                # Bottom surface vertex (extruded down)
                bottom_vertex = Point()
                bottom_vertex.x = float(point[0])
                bottom_vertex.y = float(point[1])
                bottom_vertex.z = float(point[2]) - thickness
                bottom_vertices.append(bottom_vertex)
                mesh.vertices.append(bottom_vertex)
            
            num_points = len(points)
            
            # Create top surface triangles
            for simplex in tri.simplices:
                triangle = MeshTriangle()
                # Top vertices are at indices 0, 2, 4, 6, ... (even indices)
                triangle.vertex_indices = [int(simplex[0] * 2), 
                                          int(simplex[1] * 2), 
                                          int(simplex[2] * 2)]
                mesh.triangles.append(triangle)
                
                # Bottom surface triangles (reversed winding for correct normals)
                triangle_bottom = MeshTriangle()
                triangle_bottom.vertex_indices = [int(simplex[0] * 2 + 1), 
                                                  int(simplex[2] * 2 + 1),
                                                  int(simplex[1] * 2 + 1)]
                mesh.triangles.append(triangle_bottom)
            
            # Create side triangles (connecting top and bottom)
            # Find boundary edges of the triangulation
            edges = set()
            edge_count = {}
            
            for simplex in tri.simplices:
                for i in range(3):
                    edge = tuple(sorted([simplex[i], simplex[(i+1)%3]]))
                    edge_count[edge] = edge_count.get(edge, 0) + 1
            
            # Boundary edges appear only once
            boundary_edges = [edge for edge, count in edge_count.items() if count == 1]
            
            for edge in boundary_edges:
                v0, v1 = edge
                # Create two triangles for each boundary edge
                # Triangle 1
                tri1 = MeshTriangle()
                tri1.vertex_indices = [v0*2, v1*2, v0*2+1]
                mesh.triangles.append(tri1)
                
                # Triangle 2
                tri2 = MeshTriangle()
                tri2.vertex_indices = [v1*2, v1*2+1, v0*2+1]
                mesh.triangles.append(tri2)
            
            rospy.loginfo(f"Created solid mesh with {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles (thickness={thickness}m)")
            
        except Exception as e:
            rospy.logerr(f"Error creating mesh: {e}")
            # Fallback: create a simple bounding box
            mesh = self.create_bounding_box_mesh(points)
        
        return mesh
    
    def create_bounding_box_mesh(self, points):
        """Fallback: create a simple box mesh around the points"""
        mesh = Mesh()
        
        min_point = np.min(points, axis=0)
        max_point = np.max(points, axis=0)
        
        rospy.logwarn(f"Using bounding box fallback: min={min_point}, max={max_point}")
        
        # Create 8 vertices of a box
        vertices = [
            [min_point[0], min_point[1], min_point[2]],
            [max_point[0], min_point[1], min_point[2]],
            [max_point[0], max_point[1], min_point[2]],
            [min_point[0], max_point[1], min_point[2]],
            [min_point[0], min_point[1], max_point[2]],
            [max_point[0], min_point[1], max_point[2]],
            [max_point[0], max_point[1], max_point[2]],
            [min_point[0], max_point[1], max_point[2]],
        ]
        
        for v in vertices:
            point = Point()
            point.x = float(v[0])
            point.y = float(v[1])
            point.z = float(v[2])
            mesh.vertices.append(point)
        
        # Create 12 triangles (2 per face of the box)
        triangles = [
            [0, 1, 2], [0, 2, 3],  # Bottom
            [4, 6, 5], [4, 7, 6],  # Top
            [0, 5, 1], [0, 4, 5],  # Front
            [2, 7, 3], [2, 6, 7],  # Back
            [0, 3, 7], [0, 7, 4],  # Left
            [1, 5, 6], [1, 6, 2],  # Right
        ]
        
        for t in triangles:
            triangle = MeshTriangle()
            triangle.vertex_indices = t
            mesh.triangles.append(triangle)
        
        return mesh
    
    def create_identity_pose(self):
        """Create an identity pose (no transformation)"""
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation = Quaternion(0, 0, 0, 1)
        return pose

    def _load_robot_spawn_transform(self, config_path):
        """Load robot spawn pose from scene config to transform world to base frame"""
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
                # euler_matrix yields base->world; transpose to get world->base
                base_to_world = euler_matrix(roll, pitch, yaw)[:3, :3]
                rotation = base_to_world.T
                rospy.loginfo(f"Loaded robot spawn pose from {config_path}: translation={translation}, rpy=({roll}, {pitch}, {yaw})")
        except Exception as error:
            rospy.logwarn(f"Failed to load robot spawn pose from {config_path}: {error}. Using identity transform.")

        return translation, rotation


if __name__ == '__main__':
    try:
        converter = PointCloudToCollision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
