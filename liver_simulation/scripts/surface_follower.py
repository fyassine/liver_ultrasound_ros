import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import os
import sys
import rosbag

GREEN = '\033[92m'
RESET = '\033[0m'

def log_success(msg):
    """Log success messages in green"""
    rospy.loginfo(f"{GREEN}{msg}{RESET}")

class SurfaceFollower:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('surface_follower', anonymous=True)

        robot_name = rospy.get_param('~robot_name', 'iiwa')
        ns = f"/{robot_name.strip('/')}"
        self.robot = moveit_commander.RobotCommander(robot_description="robot_description", ns=ns)
        self.scene = moveit_commander.PlanningSceneInterface(ns=ns)
        
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name, robot_description=f"/{robot_name}/robot_description", ns=robot_name)
        
        self.move_group.set_end_effector_link("probe_link_ee")
        
        self.probe_length = rospy.get_param(f"{ns}/probe_length", 0.24)
        self.surface_clearance = rospy.get_param('~surface_clearance', 0.01)
        self.contact_push = rospy.get_param('~contact_push', 0.002)
        rospy.loginfo(
            f"Using probe length: {self.probe_length}, surface clearance: {self.surface_clearance}, contact push: {self.contact_push}"
        )
        
        self.move_group.set_planning_time(10.0)
        rospy.sleep(2.0)

        self.liver_object = None
        self.find_liver_phantom()

    def find_liver_phantom(self):
        rospy.loginfo("Waiting for liver_phantom to appear in the planning scene...")
        while 'liver_phantom' not in self.scene.get_known_object_names() and not rospy.is_shutdown():
            rospy.sleep(0.5)
        
        if rospy.is_shutdown():
            return

        self.liver_object = self.scene.get_objects(['liver_phantom'])['liver_phantom']
    
    def follow_surface(self):
        if not self.liver_object:
            rospy.logerr("Liver phantom not found. Aborting surface following.")
            return

        liver_pose = self.liver_object.pose
        liver_dims = self.liver_object.primitives[0].dimensions
                
        rospy.sleep(1)

        waypoints = []

        # Liver dimensions: [0.2, 0.2, 0.1]
        liver_length = liver_dims[0]
        liver_width = liver_dims[1]
        half_length = liver_length / 2.0
        half_width = liver_width / 2.0

        rospy.loginfo("Moving probe to liver surface center with probe pointing down...")
        liver_top_z = liver_pose.position.z + (liver_dims[2] / 2.0)

        approach_pose = Pose()
        approach_pose.position.x = liver_pose.position.x
        approach_pose.position.y = liver_pose.position.y
        approach_pose.position.z = liver_top_z + self.surface_clearance + 0.3
        approach_pose.orientation.x = 0.0
        approach_pose.orientation.y = 1.0
        approach_pose.orientation.z = 0.0
        approach_pose.orientation.w = 0.0
        
        self.move_group.set_pose_target(approach_pose)
        planned_trajectory = self.move_group.plan()

        plan_ok = False
        plan_obj = None
        planning_time = None
        error_code = None

        try:
            plan_ok, plan_obj, planning_time, error_code = planned_trajectory
        except (ValueError, TypeError):
            try:
                plan_ok, plan_obj = planned_trajectory
            except (ValueError, TypeError):
                plan_obj = planned_trajectory
                plan_ok = bool(getattr(plan_obj, 'joint_trajectory', plan_obj))
        rospy.loginfo(f"plan_ok={plan_ok}, planning_time={planning_time}, error_code={getattr(error_code, 'val', error_code)}")
        if not plan_ok or not plan_obj or not hasattr(plan_obj, 'joint_trajectory') or not plan_obj.joint_trajectory.points:
            rospy.logerr("Planning failed or returned unexpected/empty trajectory. Aborting.")
            self.move_group.clear_pose_targets()
            return
        rospy.loginfo(f"Planned trajectory contains {len(plan_obj.joint_trajectory.points)} joint points")

        exec_success = self.move_group.execute(plan_obj, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        if exec_success:
            log_success("Successfully positioned probe above liver!")
            rospy.sleep(1)
        else:
            rospy.logerr("Failed to execute planned trajectory to liver surface. Aborting.")
            return

        bag = rosbag.Bag('saved_trajectory.bag', 'w')
        try:
            rospy.loginfo(f"Current working directory: {os.getcwd()}")
            rospy.loginfo(f"Saving trajectory of type: {type(plan_obj.joint_trajectory)}")
            bag.write('/iiwa/PositionJointInterface_trajectory_controller/command', plan_obj.joint_trajectory)
            rospy.loginfo("Trajectory saved to saved_trajectory.bag")
        except Exception as e:
            rospy.logerr(f"Failed to save trajectory: {e}")
        finally:
            bag.close()

if __name__ == '__main__':
    try:
        follower = SurfaceFollower()
        follower.follow_surface()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Shutting down surface follower node.")
        moveit_commander.roscpp_shutdown()