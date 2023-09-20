#!/usr/bin/env python3
import rospy
import sys
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from sensor_msgs.msg import JointState


class RobotiqGripperJointStateConverter:
    def __init__(self, name, joint_topic, joint_min, joint_max, joint_name, input_ns):
        rospy.init_node(name)
        rospy.Subscriber(input_ns + '/Robotiq2FGripperRobotInput', Robotiq2FGripper_robot_input, self.callback)
        self.pub = rospy.Publisher(joint_topic, JointState, queue_size=1)
        self.min = joint_min
        self.max = joint_max
        self.state = JointState()
        self.state.name.append(joint_name)
        self.state.position.append(0)

    def callback(self, msg: Robotiq2FGripper_robot_input):
        position = msg.gPO # Finger position 0-255
        joint_angle = self.min + (position / 255 * (self.max - self.min))
        self.state.position[0] = joint_angle
        self.pub.publish(self.state)


if __name__ == '__main__':
    args = sys.argv
    RobotiqGripperJointStateConverter(name=args[1],
                                      joint_topic=args[2],
                                      joint_min=float(args[3]),
                                      joint_max=float(args[4]),
                                      joint_name=args[5],
                                      input_ns=args[6])
    rospy.spin()

