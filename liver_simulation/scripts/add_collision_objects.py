#!/usr/bin/python3
"""
Scene Publisher for MoveIt!
Reads scene objects from YAML config and publishes them to the MoveIt!
"""

import rospy
import yaml
import os
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

# ANSI color codes for terminal output
GREEN = '\033[92m'
RESET = '\033[0m'

def log_success(msg):
    """Log success messages in green"""
    rospy.loginfo(f"{GREEN}{msg}{RESET}")


class ScenePublisher:
    def __init__(self):
        rospy.init_node('scene_publisher', anonymous=True)
        
        robot_name = rospy.get_param('~robot_name', 'iiwa')
        start_delay = rospy.get_param('~start_delay', 5.0)
        
        rospy.loginfo(f"Waiting {start_delay} seconds for move_group to start...")
        rospy.sleep(start_delay)
        
        planning_scene_topic = f'/{robot_name}/planning_scene'
        rospy.loginfo(f"Publishing to: {planning_scene_topic}")
        self.scene_pub = rospy.Publisher(planning_scene_topic, PlanningScene, queue_size=10, latch=True)
        
        rospy.sleep(1.0)
        
        self.publish_scene()
        
        log_success("Scene publisher initialized and objects published to planning scene")
    
    def load_scene_config(self):
        """Load scene objects from YAML configuration file."""
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_file = os.path.join(package_path, 'config', 'scene_objects.yaml')
        
        rospy.loginfo(f"Loading scene objects from: {config_file}")
        
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        return config.get('scene_objects', [])
    
    def create_collision_object(self, obj_config):
        """Create a CollisionObject from configuration."""
        collision_object = CollisionObject()
        collision_object.header.frame_id = obj_config.get('frame_id', 'world')
        collision_object.id = obj_config['name']
        
        # Create the shape
        primitive = SolidPrimitive()
        if obj_config['type'] == 'box':
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = obj_config['dimensions']  # [x, y, z]
        elif obj_config['type'] == 'sphere':
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [obj_config['dimensions'][0]]  # [radius]
        elif obj_config['type'] == 'cylinder':
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = obj_config['dimensions']  # [height, radius]
        
        pose = PoseStamped()
        pose.header.frame_id = obj_config.get('frame_id', 'world')
        pose.pose.position.x = obj_config['position'][0]
        pose.pose.position.y = obj_config['position'][1]
        pose.pose.position.z = obj_config['position'][2]
        pose.pose.orientation.x = obj_config['orientation'][0]
        pose.pose.orientation.y = obj_config['orientation'][1]
        pose.pose.orientation.z = obj_config['orientation'][2]
        pose.pose.orientation.w = obj_config['orientation'][3]
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose.pose)
        collision_object.operation = CollisionObject.ADD
        
        return collision_object
    
    def publish_scene(self):
        """Load and publish all scene objects to the planning scene."""
        scene_objects = self.load_scene_config()
        
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        
        for obj_config in scene_objects:
            collision_object = self.create_collision_object(obj_config)
            planning_scene.world.collision_objects.append(collision_object)
            rospy.loginfo(f"Added collision object: {obj_config['name']}")
        
        self.scene_pub.publish(planning_scene)
        log_success(f"Published {len(scene_objects)} objects to planning scene")


if __name__ == '__main__':
    try:
        scene_publisher = ScenePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
