import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import numpy as np
import csv

class MockCatheterNode(Node):
    def __init__(self):
        super().__init__('mock_catheter_node')
        self.publisher_ = self.create_publisher(Marker, 'catheter_marker', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.data = self.load_tracking_data('/home/paulstefandidraga/catheter_ws/tracking_data_matrix.csv')
        self.index = 0
        self.active_points = []
    
    def load_tracking_data(self, filename):
        data = []
        with open(filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                data.append((float(row['Position_X']), float(row['Position_Y'])))
        return data
    def timer_callback(self):
        window_size = len(self.data) // 4  # Show last 10% of data points
        if self.index < len(self.data):
            x, y = self.data[self.index]
            new_point = Point()
            new_point.x = x
            new_point.y = y
            new_point.z = 0.0

            # 4. Add the new point to the end of our active list
            self.active_points.append(new_point)

            # 5. THE MAGIC: If our list is longer than 1/3 of the circle, delete the oldest point (index 0)
            if len(self.active_points) > window_size:
                self.active_points.pop(0)

            # 6. Create TUNNEL
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "catheter_viz"
            
            # We give it ID = 0 every single time. 
            # This tells RViz: "Erase the last list I gave you and draw this new one instead!"
            marker.id = 0 
            
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD

            # Size and Color
            marker.scale.x = 0.50
            marker.scale.y = 0.50
            marker.scale.z = 0.50
            marker.color.r = 1.0
            marker.color.a = 0.5 

            # 7. Attach our carefully sized list of points to the marker
            marker.points = self.active_points

            # 8. Publish the moving segment!
            self.publisher_.publish(marker)
            
            #self.get_logger().info(f'Published point: ({x:.2f}, {y:.2f}) with {len(self.active_points)} active points.')

            # Create LINE 
            line = Marker()
            line.header.frame_id = "map"
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns = "catheter_visualization"
            
            line.id = 1  # <--- ID 1 (Crucial! Do not overwrite the tunnel)
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD

            # Thin and Solid (The Wire)
            line.scale.x = 0.1 # Only 1cm thick
            # (Remember, y and z scale don't matter for LINE_STRIP)
            
            # Let's make the centerline bright Blue so it pops against the red tunnel
            line.color.r = 0.0
            line.color.g = 0.0
            line.color.b = 1.0
            line.color.a = 1.0 

            # Re-use the exact same list of points!
            line.points = self.active_points

            # Publish the line immediately after!
            self.publisher_.publish(line)
            self.index += 1
        else:
            self.get_logger().info('Data Looped.')
            self.index = 0

def main(args=None):
    rclpy.init(args=args)
    mock_catheter_node = MockCatheterNode()
    rclpy.spin(mock_catheter_node)
    mock_catheter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()