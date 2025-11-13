#!/usr/bin/env python3

import rospy
from iiwa_msgs.msg import CartesianPose, CartesianWrench, JointPosition, JointVelocity, JointTorque
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState


class IiwaToStandardRepublisher:
    def __init__(self):
        rospy.init_node('iiwa_to_standard_republisher', anonymous=True)

        self.cartesian_pose_pub = rospy.Publisher('/iiwa/state/CartesianPose_standard', PoseStamped, queue_size=10)
        self.cartesian_wrench_pub = rospy.Publisher('/iiwa/state/CartesianWrench_standard', WrenchStamped, queue_size=10)
        self.joint_position_pub = rospy.Publisher('/iiwa/state/JointPosition_standard', JointState, queue_size=10)
        self.joint_velocity_pub = rospy.Publisher('/iiwa/state/JointVelocity_standard', JointState, queue_size=10)
        self.joint_torque_pub = rospy.Publisher('/iiwa/state/JointTorque_standard', JointState, queue_size=10)

        rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, self.cartesian_pose_callback)
        rospy.Subscriber('/iiwa/state/CartesianWrench', CartesianWrench, self.cartesian_wrench_callback)
        rospy.Subscriber('/iiwa/state/JointPosition', JointPosition, self.joint_position_callback)
        rospy.Subscriber('/iiwa/state/JointVelocity', JointVelocity, self.joint_velocity_callback)
        rospy.Subscriber('/iiwa/state/JointTorque', JointTorque, self.joint_torque_callback)

        rospy.loginfo("iiwa to standard republisher started")

    def cartesian_pose_callback(self, cartesian_pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped = cartesian_pose.poseStamped
        self.cartesian_pose_pub.publish(pose_stamped)

    
    def cartesian_wrench_callback(self, cartesian_wrench):
        wrench_stamped = WrenchStamped()
        wrench_stamped.header.stamp = rospy.Time.now()
        wrench_stamped.header.frame_id = "base_link"
        wrench_stamped.wrench = cartesian_wrench.wrench
        self.cartesian_wrench_pub.publish(wrench_stamped)

    def joint_position_callback(self, joint_position):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4',
                           'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
        joint_state.position = [
            joint_position.position.a1, joint_position.position.a2, joint_position.position.a3,
            joint_position.position.a4, joint_position.position.a5, joint_position.position.a6,
            joint_position.position.a7
        ]
        self.joint_position_pub.publish(joint_state)

    def joint_velocity_callback(self, joint_velocity):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4',
                           'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
        joint_state.velocity = [
            joint_velocity.velocity.a1, joint_velocity.velocity.a2, joint_velocity.velocity.a3,
            joint_velocity.velocity.a4, joint_velocity.velocity.a5, joint_velocity.velocity.a6,
            joint_velocity.velocity.a7
        ]
        self.joint_velocity_pub.publish(joint_state)

    def joint_torque_callback(self, joint_torque):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4',
                           'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
        joint_state.effort = [
            joint_torque.torque.a1, joint_torque.torque.a2, joint_torque.torque.a3,
            joint_torque.torque.a4, joint_torque.torque.a5, joint_torque.torque.a6,
            joint_torque.torque.a7
        ]
        self.joint_torque_pub.publish(joint_state)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        republisher = IiwaToStandardRepublisher()
        republisher.spin()
    except rospy.ROSInterruptException:
        pass
