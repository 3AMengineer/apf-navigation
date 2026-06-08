import rclpy
import numpy as np
import math

from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class ControllerNode(Node): 
    
    def __init__(self):
        super().__init__('controller_node')
        
        self.bridge = CvBridge() # Helper for image conversion Initialized once here for low computation 

        self.subscription_odom = self.create_subscription(
            Odometry, '/odom', self.listener_callback_odom, 10)

        self.subscription_depth = self.create_subscription(
            Image, '/camera/depth/image_raw', self.listener_callback_depth, 10)

        self.controller_timer = self.create_timer(0.1, self.apfa_controller) 
        
        self.command_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.odom_data = None
        self.depth_data = None 
        #self.lidar_data_time = time.time() 

        # D455 Camera Parameters
        self.fov_horizontal_deg = 87.0
        self.fov_horizontal_rad = np.deg2rad(self.fov_horizontal_deg)
        self.camera_max_dist = 5.0 # Clip distances beyond this
        self.camera_min_dist = 0.05 # Blind spot of D455

    def listener_callback_odom(self, odom_data):       
        self.odom_data = odom_data
    
    # CHANGED: Callback now handles Image
    def listener_callback_depth(self, msg):
        self.depth_data = msg
        #self.lidar_data_time = time.time()

    def apfa_controller(self):
        # Safety check
        if self.depth_data is None or self.odom_data is None:
            return



        cv_image = self.bridge.imgmsg_to_cv2(self.depth_data, desired_encoding='passthrough')
        depth_array = np.array(cv_image, dtype=np.float32)


        # Handle NaNs and Infs (common in simulation)
        depth_array = np.nan_to_num(depth_array, nan=10.0, posinf=10.0)

        # Vertical ROI (Region of Interest) - Crucial for "Ground Removal"
        # We only look at the center slice of the image to ignore the floor
        height, width = depth_array.shape # mXn matrix = wXh
        center_row = height // 2
        roi_height = 60 # Look at a 60-pixel tall strip in the middle # My region of interest
        
        # Crop the image to the strip # limiting my FOV
        depth_strip = depth_array[center_row - roi_height//2 : center_row + roi_height//2,:]

        # "Collapse" the strip: Find closest obstacle in every vertical column
        # This creates a 1D array effectively identical to 'lidar_ranges'
        virtual_scan = np.min(depth_strip, axis=0)

        # Downsampling: RealSense has ~848 columns. 
        # Your clustering loop is O(N). We skip pixels to keep it fast.
        #step = 5
        #virtual_scan = virtual_scan[::step] 
        num_points = len(virtual_scan)

        # Precompute angles for the virtual scan
        # Camera Frame: Left is +Angle, Right is -Angle (Standard ROS convention)
        angles = np.linspace(self.fov_horizontal_rad/2, -self.fov_horizontal_rad/2, num_points)

        points_obstacle = [] 
        
        for i, dist_obstacle in enumerate(virtual_scan):
            
            # Filter noise and limits
            #if dist_obstacle < self.camera_min_dist or dist_obstacle > self.camera_max_dist:
             #   continue

            angle_obstacle = angles[i] 
            
            # Conversion: Polar to Cartesian (Body Frame)
            # Robot X = Forward, Robot Y = Left
            x = dist_obstacle * math.cos(angle_obstacle) 
            y = dist_obstacle * math.sin(angle_obstacle) 
            
            points_obstacle.append((x, y, dist_obstacle, angle_obstacle))

    
        clusters = [] 
        current_cluster = []
        CLUSTER_DIST = 0.5 # Increased slightly for depth noise
        MIN_CLUSTER_SIZE = 2

        for i in range(len(points_obstacle)):
            if len(current_cluster) == 0: 
                current_cluster.append(points_obstacle[i]) 
                continue
            
            x1, y1, _, _ = points_obstacle[i - 1] 
            x2, y2, _, _ = points_obstacle[i] 

            if math.hypot(x2 - x1, y2 - y1) < CLUSTER_DIST:
                current_cluster.append(points_obstacle[i]) 
            else:
                if len(current_cluster) >= MIN_CLUSTER_SIZE:
                    clusters.append(current_cluster)
                current_cluster = [points_obstacle[i]] 

        if len(current_cluster) > 0:
            clusters.append(current_cluster)

        obstacles_closest_point = [] 

        for cluster in clusters:
            closest_point = cluster[0]               
            for p in cluster:
                if p[2] < closest_point[2]:           
                    closest_point = p
            obstacles_closest_point.append(closest_point)           


        x_odom = self.odom_data.pose.pose.position.x
        y_odom = self.odom_data.pose.pose.position.y
        quat_x = self.odom_data.pose.pose.orientation.x
        quat_y = self.odom_data.pose.pose.orientation.y
        quat_z = self.odom_data.pose.pose.orientation.z
        quat_w = self.odom_data.pose.pose.orientation.w
        
        # Yaw calculation
        yaw = math.atan2(2.0 * (quat_w * quat_z + quat_x * quat_y), 1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z))
        
        goal_position = [-6.0, -6.0] # Ensure floats
        current_position = [x_odom, y_odom]

        ka = 0.5
        kr = 500


        dx = goal_position[0] - x_odom 
        dy = goal_position[1] - y_odom 

        distance_error = math.dist(goal_position, current_position)
        
        msg = Twist()

        if distance_error < 0.35:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            self.command_vel_pub.publish(msg)
            return        
        
        # Attractive Force
        dist_vector = np.array([[dx],[dy]])
        force_attr_vector = ka * dist_vector
        # force_attr_magnitude = ka * math.sqrt(dx*dx + dy*dy)

        # Repulsive Force
        Frep_radius = 4.5 # INCREASED: Camera is blind to sides, need earlier reaction
        force_rep_x = 0.0
        force_rep_y = 0.0

        if len(obstacles_closest_point) > 0:
            for obs in obstacles_closest_point:
                _, _, distance_obs, theta_obs = obs 

                if distance_obs > Frep_radius:
                    continue 

                # Standard APF Repulsive Magnitude
                # Added max cap to prevent infinity at dist=0
                force_reps_magnitude = kr * (1.0 / distance_obs - 1.0 / Frep_radius) / (distance_obs ** 2)

                # Convert obstacle angle (Body Frame) to World Frame for force summation
                angle_world = theta_obs + yaw

                force_rep_x += -force_reps_magnitude * math.cos(angle_world)
                force_rep_y += -force_reps_magnitude * math.sin(angle_world)

        force_reps_Vector = np.array([[force_rep_x], [force_rep_y]])

        # Resultant
        force_resultant = force_reps_Vector + force_attr_vector
        
        desired_yaw = math.atan2(force_resultant[1,0], force_resultant[0,0])
        yaw_error = desired_yaw - yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        speed = math.sqrt(force_resultant[0,0]**2 + force_resultant[1,0]**2)

        # Apply limits
        msg.linear.x = min(0.3 , speed)
        msg.angular.z = 0.96* yaw_error # Increased gain for faster turning

        self.command_vel_pub.publish(msg)

def main(args=None):    
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()