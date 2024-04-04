#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('playground_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 1)
        self.subscription = self.create_subscription(
            JointState,
            '/lbr/joint_states',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose

# class GoalPublisher(Node):

#     def __init__(self):
#         super().__init__('goal_publisher')
#         self.publisher_ = self.create_publisher(Pose, 'lbr/moveit_goal', 10)
#         msg = Pose()
#         msg.position.x = -0.4
#         msg.position.y = -0.08
#         msg.position.z = 0.4
#         msg.orientation.x = 0.9063
#         msg.orientation.y = 0.0
#         msg.orientation.z = 0.0
#         msg.orientation.w = 0.4226
#         self.publisher_.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     goal_publisher = GoalPublisher()
#     # rclpy.spin(goal_publisher)
#     goal_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()