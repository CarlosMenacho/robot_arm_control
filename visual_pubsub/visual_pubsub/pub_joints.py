import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time


class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(
            0.1, self.publish_joint_states)  # Publish every 0.1s

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
        msg.name = [
            'joint_a1', 'joint_a2', 'joint_a3', 'joint_a4',
            'joint_a5', 'joint_a6'
        ]
        msg.position = [0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0]

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published Joint States: {msg.position}')  # Debugging info


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
