import rclpy
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
        
        self.active_points = []
        
        # 3. Since data is now infinite, we set a hard limit on the tail length.
        # The ESP32 publishes every 100ms (10Hz). 60 points = 6 seconds of tail.
        self.window_size = 60 

    def listener_callback(self, msg):
        # Safety check: ensure the array has at least an X and Y
        if len(msg.data) >= 2:
            x = float(msg.data[0])
            y = float(msg.data[1])
        else:
            self.get_logger().warn('Received malformed data array from ESP32')
            return

        # 4. Create the new point
        new_point = Point()
        new_point.x = x
        new_point.y = y
        new_point.z = 0.0

        # 5. Add the new point to the end of our active list
        self.active_points.append(new_point)

        # 6. If our list is longer than the window size, delete the oldest point (index 0)
        if len(self.active_points) > self.window_size:
            self.active_points.pop(0)

        # 7. Create TUNNEL Marker
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

        # 8. Create LINE Marker
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