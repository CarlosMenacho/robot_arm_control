import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import numpy as np
from numpy import sin, cos
import sympy as sp
import time


class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.target_sub = self.create_subscription(Point, 'target_position',
                                                   self.target_callback, 10)
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        # Initial joint angles
        self.q = np.array([0, 
                           -1.5708, 
                           1.5708, 
                           0, 
                           0, 
                           0])
        
        self.joint_names = ["joint_a1",
                            "joint_a2", 
                            "joint_a3", 
                            "joint_a4", 
                            "joint_a5", 
                            "joint_a6"]


        self.timer = self.create_timer(0.1, self.update_joints)
        self.marker_timer = self.create_timer(1.0, self.publish_marker)
        self.target_pos = np.array([1.2, 0.5, 0.0])  # Default target

        # Parameters for IK
        self.step_size = 0.05
        self.max_iterations = 100
        self.tolerance = 0.01
        self.damping_factor = 0.1  # For damped least squares method

    def publish_marker(self):
        marker = Marker()
        
        # 1. Reference Frame and Timestamp
        marker.header.frame_id = "base"  
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # 2. Marker Identification
        marker.ns = "goal"
        marker.id = 0
        
        # 3. Shape and Action
        marker.type = Marker.SPHERE  # Options: CUBE, CYLINDER, ARROW, etc.
        marker.action = Marker.ADD
        
        # 4. Position and Orientation in space
        marker.pose.position.x = self.target_pos[0]
        marker.pose.position.y = self.target_pos[1]
        marker.pose.position.z = self.target_pos[2]
        marker.pose.orientation.w = 1.0  # Default unrotated quaternion
        
        # 5. Scale (Size in meters)
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # 1.0 is fully opaque, 0.0 is invisible
        
        self.publisher.publish(marker)

    def forward_kinematics(self, q):

        q1 = q[0]
        q2 = q[1] + np.pi/2 
        q3 = q[2] - np.pi/2  
        q4 = q[3]
        q5 = q[4]
        q6 = q[5]
        x = 0.215*sin(q1)*sin(q4)*sin(q5) + 1.15*sin(q2)*cos(q1) - 0.215*sin(q5)*sin(
            q2 + q3)*cos(q1)*cos(q4) + 0.215*cos(q1)*cos(q5)*cos(q2 + q3) + 1.2*cos(q1)*cos(
                q2 + q3) + 0.35*cos(q1)

        y = 1.15*sin(q1)*sin(q2) - 0.215*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) + 0.215*sin(
            q1)*cos(q5)*cos(q2 + q3) + 1.2*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) - 0.215*sin(
                q4)*sin(q5)*cos(q1)

        z = -0.215*sin(q5)*cos(q4)*cos(q2 + q3) - 0.215*sin(q2 + q3)*cos(q5) - 1.2*sin(
            q2 + q3) + 1.15*cos(q2) + 0.675

        return np.array([x, y, z])

    def jacobian(self, q):
        q1, q2, q3, q4, q5, q6 = q

        q1 = q[0]
        q2 = q[1] + np.pi/2 
        q3 = q[2] - np.pi/2  
        q4 = q[3]
        q5 = q[4]
        q6 = q[5]

        j11 = -1.15*np.sin(q1)* np.sin(q2) + 0.215*np.sin(
            q1)*np.sin(q5)*np.sin(q2 + q3)*np.cos(q4) - 0.215*np.sin(
            q1)*np.cos(q5)*np.cos(q2 + q3) - 1.2*np.sin(q1)*np.cos(
                q2 + q3) - 0.35*np.sin(q1) + 0.215*np.sin(q4)*np.sin(q5)*np.cos(q1)
        
        j12 = -0.215*sin(q5)*cos(q1)*cos(q4)*cos(q2 + q3) - 0.215*sin(q2 + q3)*cos(
            q1)*cos(q5) - 1.2*sin(q2 + q3)*cos(q1) + 1.15*cos(q1)*cos(q2)
        
        j13 = -0.215*sin(q5)*cos(q1)*cos(q4)*cos(q2 + q3) - 0.215*sin(q2 + q3)*cos(
            q1)*cos(q5) - 1.2*sin(q2 + q3)*cos(q1)

        j14 = 0.215*sin(q1)*sin(q5)*cos(q4) + 0.215*sin(q4)*sin(q5)*sin(q2 + q3)*cos(q1)

        j15 = 0.215*sin(q1)*sin(q4)*cos(q5) - 0.215*sin(q5)*cos(q1)*cos(q2 + q3) - 0.215*sin(
            q2 + q3)*cos(q1)*cos(q4)*cos(q5)
        j16 = 0


        j21 = 0.215*sin(q1)*sin(q4)*sin(q5) + 1.15*sin(q2)*cos(q1) - 0.215*sin(q5)*sin(q2 + q3)*cos(
            q1)*cos(q4) + 0.215*cos(q1)*cos(q5)*cos(q2 + q3) + 1.2*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)

        j22 = -0.215*sin(q1)*sin(q5)*cos(q4)*cos(q2 + q3) - 0.215*sin(q1)*sin(q2 + q3)*cos(
            q5) - 1.2*sin(q1)*sin(q2 + q3) + 1.15*sin(q1)*cos(q2)
        
        j23 = -0.215*sin(q1)*sin(q5)*cos(q4)*cos(q2 + q3) - 0.215*sin(q1)*sin(
            q2 + q3)*cos(q5) - 1.2*sin(q1)*sin(q2 + q3)

        j24 = 0.215*sin(q1)*sin(q4)*sin(q5)*sin(q2 + q3) - 0.215*sin(q5)*cos(q1)*cos(q4)
        j25 = -0.215*sin(q1)*sin(q5)*cos(q2 + q3) - 0.215*sin(q1)*sin(q2 + q3)*cos(q4)*cos(
            q5) - 0.215*sin(q4)*cos(q1)*cos(q5)
        j26=0


        j31 = 0
        j32 = -1.15*sin(q2) + 0.215*sin(q5)*sin(q2 + q3)*cos(q4) - 0.215*cos(q5)*cos(q2 + q3) - 1.2*cos(q2 + q3)
        j33 = 0.215*sin(q5)*sin(q2 + q3)*cos(q4) - 0.215*cos(q5)*cos(q2 + q3) - 1.2*cos(q2 + q3)
        j34 = 0.215*sin(q4)*sin(q5)*cos(q2 + q3)
        j35 = 0.215*sin(q5)*sin(q2 + q3) - 0.215*cos(q4)*cos(q5)*cos(q2 + q3)
        j36 = 0

        return np.array([[j11, j12, j13, j14, j15, j16], 
                         [j21, j22, j23, j24, j25, j26], 
                         [j31, j32, j33, j34, j35, j36]])

    def target_callback(self, msg):
        self.target_pos = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(
            f"New target received: [{msg.x}, {msg.y}, {msg.z}]")

    def update_joints(self):
        current_pos = self.forward_kinematics(self.q)
        error = self.target_pos - current_pos
        error_norm = np.linalg.norm(error)

        self.get_logger().info(f"Current position: {current_pos}")
        self.get_logger().info(f"Target position: {self.target_pos}")
        self.get_logger().info(f"Error: {error_norm}")

        if error_norm > self.tolerance:
            J = self.jacobian(self.q)

            J_sq = J @ J.T
            manipulability = np.sqrt(np.linalg.det(J_sq))
            if manipulability < 0.01:
                self.get_logger().warning(f"close to singularity: {manipulability}")

            JtJ = J.T @ J
            damping = self.damping_factor * np.eye(JtJ.shape[0])
            J_dls = np.linalg.solve(JtJ + damping, J.T) @ error

            # Apply update with step size
            self.q += J_dls * self.step_size

        # Publish updated joint states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base"  # Add this line to fix the empty frame_id error
        msg.name = self.joint_names
        msg.position = self.q.tolist()
        self.joint_pub.publish(msg)


def main():
    rclpy.init()
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
