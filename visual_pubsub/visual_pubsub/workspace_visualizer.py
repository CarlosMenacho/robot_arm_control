import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from numpy import sin, cos

class WorkspaceVisualizer(Node):
    def __init__(self):
        super().__init__('workspace_visualizer')
        self.publisher = self.create_publisher(Marker, 'workspace_marker', 10)
        self.timer = self.create_timer(2.0, self.publish_workspace)
        
        self.get_logger().info("Generating workspace points using Monte Carlo simulation...")
        
        self.points = self.generate_workspace(num_samples=50000)
        self.get_logger().info(f"Generated {len(self.points)} points. Publishing to RViz...")

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

    def generate_workspace(self, num_samples):
        points = []
        q_random = np.random.uniform(-np.pi, np.pi, (num_samples, 6))
        
        for q in q_random:
            pos = self.forward_kinematics(q)
            
            if pos[2] < 0.0:
                continue
                
            p = Point()
            p.x = float(pos[0])
            p.y = float(pos[1])
            p.z = float(pos[2])
            points.append(p)
            
        return points

    def publish_workspace(self):
        marker = Marker()
        marker.header.frame_id = "base"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "workspace"
        marker.id = 1
        
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.5
        
        marker.points = self.points
        
        self.publisher.publish(marker)
        
def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
