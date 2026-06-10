'''
This node recieves data from the ESP32, computes the catheter tip position, 
and visualizes it in RViz. 
It also publishes a joystick command to the /Joystick_data topic.
'''


import rclpy
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
# 1. Import the message type matching the ESP32
from std_msgs.msg import Float32MultiArray, Int32

class MockCatheterNode(Node):
    def __init__(self):
        super().__init__('mock_catheter_node')
        
        # Publisher for RViz
        self.publisher_ = self.create_publisher(Marker, 'catheter_marker', 10)

        # Publisher for Joystick
        self.joystick_publisher_ = self.create_publisher(Int32, '/Joystick_data', 10)
        
        # 2. Subscriber listening to the ESP32
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/catheter_floats',
            self.listener_callback,
            10
        )
        

        self.position = np.array([[0.0], [0.0], [0.0]])  # Initial position of the catheter tip
        self.R_global = np.eye(3)
        self.prev_pos = 0.0

        self.radius = 0.05 # encoder wheel radius in meters

        self.active_points = []
        
        # 3. Since data is now infinite, we set a hard limit on the tail length.
        # The ESP32 publishes every 100ms (10Hz). 60 points = 6 seconds of tail.
        self.window_size = 60

    def listener_callback(self, msg):
        # Get esp32
        if len(msg.data) >= 7:
            encoder_x = float(msg.data[0])
            omega_x = float(msg.data[1])
            omega_y = float(msg.data[2])
            omega_z = float(msg.data[3])
            tof1 = float(msg.data[4])
            tof2 = float(msg.data[5])
            tof3 = float(msg.data[6])
        else:
            self.get_logger().warn('Received malformed data array from ESP32')
            return
        #ADD FILTERING HERE 

        # Compute new position based on received data

        x_dist = encoder_x * self.radius - self.prev_pos
        self.prev_pos = encoder_x * self.radius

        V_local = np.array([[x_dist], [0.0], [0.0]])

        theta_x = omega_x * 0.1
        theta_y = omega_y * 0.1
        theta_z = omega_z * 0.1 
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(theta_x), -np.sin(theta_x)],
                        [0, np.sin(theta_x), np.cos(theta_x)]])
        R_y = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                        [0, 1, 0],
                        [-np.sin(theta_y), 0, np.cos(theta_y)]])
        R_z = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                        [np.sin(theta_z), np.cos(theta_z), 0],
                        [0, 0, 1]])
        R_local = R_z @ R_y @ R_x
        self.R_global = self.R_global @ R_local
        self.position = self.R_global @ V_local + self.position


        
        new_point = Point()
        new_point.x = self.position[0, 0]
        new_point.y = self.position[1, 0]
        new_point.z = self.position[2, 0]

        # Add the new point to the end of our active list
        self.active_points.append(new_point)

        # If our list is longer than the window size, delete the oldest point (index 0)
        if len(self.active_points) > self.window_size:
            self.active_points.pop(0)

        # Create TUNNEL Marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "catheter_viz"
        
        marker.id = 0 
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        # Size and Color
        marker.scale.x = 0.50
        marker.scale.y = 0.50
        marker.scale.z = 0.50
        marker.color.r = 1.0
        marker.color.a = 0.5 
        marker.points = self.active_points

        # Publish the moving segment!
        self.publisher_.publish(marker)

        # Create LINE Marker
        line = Marker()
        line.header.frame_id = "map"
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = "catheter_visualization"
        
        line.id = 1  
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD

        # Thin and Solid (The Wire)
        line.scale.x = 0.1 
        line.color.r = 0.0
        line.color.g = 0.0
        line.color.b = 1.0
        line.color.a = 1.0 
        line.points = self.active_points

        # Publish the line immediately after!
        self.publisher_.publish(line)

        #Publish joystick command
        joystick_msg = Int32()
        joystick_msg.data = 5
        self.joystick_publisher_.publish(joystick_msg)

def main(args=None):
    rclpy.init(args=args)
    mock_catheter_node = MockCatheterNode()
    rclpy.spin(mock_catheter_node)
    mock_catheter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()