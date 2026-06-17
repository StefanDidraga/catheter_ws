'''
This node receives data from the ESP32, computes the catheter tip position, 
and visualizes it in RViz. 
It also publishes a joystick command to the /Joystick_data topic.
'''

import rclpy
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Int32

class MockCatheterNode(Node):
    def __init__(self):
        super().__init__('mock_catheter_node')
        
        # Publisher for RViz
        self.publisher_ = self.create_publisher(Marker, 'catheter_marker', 10)

        # Publisher for Joystick
        self.joystick_publisher_ = self.create_publisher(Int32, '/Joystick_data', 10)
        
        # Subscriber listening to the ESP32
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/catheter_floats',
            self.listener_callback,
            10
        )

        self.position = np.array([[0.0], [0.0], [0.0]])  # Initial position of the catheter tip
        self.R_global = np.eye(3)
        self.prev_encoder_x = None

        self.radius = 0.05 # encoder wheel radius in meters
        self.encoder_resolution = 4096 # encoder ticks per revolution
        
        # Infinite tails
        self.active_points = []   # Centerline points (catheter trajectory)
        self.vessel_points = []   # Points forming the circular walls of the vessel
        
        # Tracking forward movement for circle dropping
        self.accumulated_forward_dist = 0.0
        self.distance_threshold = 0.0005 # 0.5 mm threshold

    def fit_circle_2d(self, y1, z1, y2, z2, y3, z3):
        """ Calculate circumcenter and radius from 3 points in a 2D plane """
        D = 2 * (y1*(z2 - z3) + y2*(z3 - z1) + y3*(z1 - z2))
        
        # If the points are essentially collinear or extremely close, default to 0 to prevent division by zero
        if abs(D) < 1e-8:
            return 0.0, 0.0, 0.001 
            
        yc = ((y1**2 + z1**2)*(z2 - z3) + (y2**2 + z2**2)*(z3 - z1) + (y3**2 + z3**2)*(z1 - z2)) / D
        zc = ((y1**2 + z1**2)*(y3 - y2) + (y2**2 + z2**2)*(y1 - y3) + (y3**2 + z3**2)*(y2 - y1)) / D
        r = np.sqrt((y1 - yc)**2 + (z1 - zc)**2)
        
        return yc, zc, r

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

        # Filtering/Offsets
        tof1 -= 16
        tof2 -= 16
        tof3 -= 16

        # Convert ToF from mm to meters for RViz standard units
        d1 = max(tof1 * 0.001, 0.0)
        d2 = max(tof2 * 0.001, 0.0)
        d3 = max(tof3 * 0.001, 0.0)

        if(omega_x < 0.01):
            omega_x = 0.0
        if(omega_y < 0.01):
            omega_y = 0.0
        if(omega_z < 0.01):
            omega_z = 0.0

        # Compute new position based on received data
        x_dist = 0.0
        if self.prev_encoder_x is None:
            self.prev_encoder_x = encoder_x
        else:
            delta_ticks = encoder_x - self.prev_encoder_x
            half_res = self.encoder_resolution / 2.0
            
            # Check for wrap-around
            if delta_ticks > half_res:
                delta_ticks -= self.encoder_resolution
            elif delta_ticks < -half_res:
                delta_ticks += self.encoder_resolution
                
            delta_rad = (delta_ticks / self.encoder_resolution) * 2 * np.pi
            x_dist = delta_rad * self.radius

        self.prev_encoder_x = encoder_x

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

        # Save the current centerline point
        new_point = Point()
        new_point.x = self.position[0, 0]
        new_point.y = self.position[1, 0]
        new_point.z = self.position[2, 0]
        self.active_points.append(new_point)

        # Track forward movement to drop vessel rings
        if x_dist > 0:
            self.accumulated_forward_dist += x_dist

        # Publish a new circle if we moved forward 1mm
        if self.accumulated_forward_dist >= self.distance_threshold:
            self.accumulated_forward_dist = 0.0 # Reset threshold tracker
            
            # Calculate coordinates of the 3 ToF hits in the local Y-Z plane
            # Radially spaced at 0, 120, and 240 degrees.
            # Point 1: 0 degrees
            y1, z1 = d1, 0.0
            # Point 2: 120 degrees
            y2, z2 = d2 * np.cos(2*np.pi/3), d2 * np.sin(2*np.pi/3)
            # Point 3: 240 degrees
            y3, z3 = d3 * np.cos(4*np.pi/3), d3 * np.sin(4*np.pi/3)

            # Recreate the circle that holds those three points
            yc, zc, r = self.fit_circle_2d(y1, z1, y2, z2, y3, z3)

            # Generate the points for this circle ring
            num_points_in_circle = 20
            angles = np.linspace(0, 2*np.pi, num_points_in_circle)
            
            # Loop creates line segments (A to B) to form a continuous ring
            for i in range(len(angles) - 1):
                yA = yc + r * np.cos(angles[i])
                zA = zc + r * np.sin(angles[i])
                yB = yc + r * np.cos(angles[i+1])
                zB = zc + r * np.sin(angles[i+1])

                pA_local = np.array([[0.0], [yA], [zA]])
                pB_local = np.array([[0.0], [yB], [zB]])

                pA_global = self.position + self.R_global @ pA_local
                pB_global = self.position + self.R_global @ pB_local

                self.vessel_points.append(Point(x=pA_global[0,0], y=pA_global[1,0], z=pA_global[2,0]))
                self.vessel_points.append(Point(x=pB_global[0,0], y=pB_global[1,0], z=pB_global[2,0]))

        # ----------------------------------------
        # Visualizing the Vessel Walls (Rings)
        wall_marker = Marker()
        wall_marker.header.frame_id = "map"
        wall_marker.header.stamp = self.get_clock().now().to_msg()
        wall_marker.ns = "vessel_visualization"
        wall_marker.id = 0 
        wall_marker.type = Marker.LINE_LIST
        wall_marker.action = Marker.ADD

        # Thin transparent green lines for the vessel rings
        wall_marker.scale.x = 0.002 # 2mm line thickness
        wall_marker.color.r = 0.0
        wall_marker.color.g = 1.0
        wall_marker.color.b = 0.0
        wall_marker.color.a = 0.4
        wall_marker.points = self.vessel_points

        self.publisher_.publish(wall_marker)

      # Visualizing the Catheter wire (Centerline)
        line = Marker()
        line.header.frame_id = "map"
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = "catheter_visualization"
        line.id = 1  
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD

        # Thin solid blue line for the catheter tracking
        line.scale.x = 0.003
        line.color.r = 0.0
        line.color.g = 0.0
        line.color.b = 1.0
        line.color.a = 1.0 
        line.points = self.active_points

        self.publisher_.publish(line)

        # Publish joystick command
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