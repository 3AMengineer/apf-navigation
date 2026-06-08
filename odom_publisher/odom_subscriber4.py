import rclpy 
import numpy as np
import time
import math


from rclpy.node import Node # node is a module inside rclpy and Node is a class inside rclpy.node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#These lines represent the node’s dependencies. Recall that dependencies have to be added to package.xml

class ControllerNode(Node): # inherited ControllerNode from Node class
    
    def __init__(self):
        super().__init__('controller_node') #Following is the definition of the class’s constructor. super().__init__ calls the Node class’s constructor and gives it your node name, in this case odom_publisher.
        
        self.subscription_odom = self.create_subscription(Odometry,'/odom',self.listener_callback_odom,10) # create_subscription(Data_type,topicName,QoS)
        #The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message.

        self.subscription_lidar = self.create_subscription(LaserScan,'/scan',self.listener_callback_lidar,10)

        self.controller_timer = self.create_timer(0.1,self.apfa_controller) 
        # We need to add a create_timer function to call the apfa_controller function periodically
        self.command_vel_pub =  self.create_publisher(Twist,'/cmd_vel',10)

        # the function create_publisher returns an object that object is equated to the variable command_vel_pub

    def listener_callback_odom(self, odom_data):
        # self.get_logger().info(odom_data.twist)   #get_logger().info is only for string.        
        self.odom_data = odom_data
        #self.odom_data_time = time.time()
    
    def listener_callback_lidar(self,lidar_data):
        self.lidar_data = lidar_data
        #self.lidar_data_time = time.time()


    def apfa_controller(self):
        # Convert LiDAR range list to a NumPy array
        lidar_ranges = np.array(self.lidar_data.ranges)
        angle_min = self.lidar_data.angle_min
        angle_increment = self.lidar_data.angle_increment

        points_obstacle = []  # (x, y, distance, angle)
        for index_lidar, dist_obstacle in enumerate(lidar_ranges): # index_lidar runs like 0,1,2,3,4,5 ...
                                                                   # dist_obstacle stored distance of obstacle from body fram
            if not np.isfinite(dist_obstacle): # if dist_obstacle is infinite it is ingnored else it starts line 52
                continue # runs loop again with i+1

            # if angle_obstacle is finite this code is run
            angle_obstacle = angle_min + index_lidar * angle_increment # from body frame
            x = dist_obstacle* math.cos(angle_obstacle) #convert polar to cartesian w.r.t body frame
            y = dist_obstacle * math.sin(angle_obstacle) #convert polar points to cartesian w.r.t body frame
            points_obstacle.append((x, y, dist_obstacle, angle_obstacle)) 



        clusters = [] # collection of nearby consecutive points
        current_cluster = []
        CLUSTER_DIST = 0.3
        MIN_CLUSTER_SIZE = 2

        for i in range(len(points_obstacle)):
            if len(current_cluster) == 0  : 
                current_cluster.append(points_obstacle[i]) # adds the point to a cluster if it is the first point of an obstacle
                continue
            
            # this code runs only if minimum one element is present in the list CURRENT_CLUSTER
            x1, y1, _, _ = points_obstacle[i - 1] # _, ignores that value i.e d and theta # comapres prvs lidar point and current lidar point
            x2, y2, _, _ = points_obstacle[i] 

            if math.hypot(x2 - x1, y2 - y1) < CLUSTER_DIST:
                current_cluster.append(points_obstacle[i]) # adds the point to CURRENT_CLUSTER if distance is less than 0.3 b/w both points bcoz it belongs to same obstacle
            else:
                if len(current_cluster) >= MIN_CLUSTER_SIZE:
                    clusters.append(current_cluster)
                current_cluster = [points_obstacle[i]] # override the prvs (current_cluster point) to check for new iteration

        if len(current_cluster) > 0:
            clusters.append(current_cluster)


        obstacles_closest_point = [] # list of closest points one per obstacle

        for cluster in clusters:
            closest_point = cluster[0]               # first point in this cluster just assumed to be closest 
            for p in cluster:
                if p[2] < closest_point[2]:           # compare distance
                    closest_point = p
            obstacles_closest_point.append(closest_point)            # one obstacle per cluster



        #print(lidar_ranges)
        #print("Obstacles detected:", len(obstacles))

        x_odom = self.odom_data.pose.pose.position.x
        y_odom = self.odom_data.pose.pose.position.y
        # z_odom = self.odom_data.pose.pose.position.z

        quat_x = self.odom_data.pose.pose.orientation.x
        quat_y = self.odom_data.pose.pose.orientation.y
        quat_z = self.odom_data.pose.pose.orientation.z
        quat_w = self.odom_data.pose.pose.orientation.w

        velocity_x = self.odom_data.twist.twist.linear.x

        ang_vel_z = self.odom_data.twist.twist.angular.z

        quaternion = [quat_x,quat_y, quat_z, quat_w]  # Replace with actual values
        
        yaw = math.atan2(2.0 * (quat_w * quat_z + quat_x * quat_y),1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z))
        
        # Setting up Values
        goal_position  = [-6,-6]
        current_position = [x_odom,y_odom]

        ka = 1.2
        kr = 80

        dx = goal_position[0] - x_odom # points from body to destination
        dy = goal_position[1] - y_odom 

        distance_error = math.dist(goal_position,current_position)
        direction_goal = math.atan2(dy,dx)
        #angle_error = direction_goal-yaw
        # Normalize angle error to [-pi, pi] so the robot takes the shortest turn
        #angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        msg = Twist()

        if distance_error < 0.35:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            self.command_vel_pub.publish(msg)
            return        
        
        #let's add VECTORS AS PER APF LOGIC
        #odom data from diff_drive plugin is very trashy but let it be considered as noise

        dist_vector = np.array([[dx],[dy]])
        force_attr_vector = ka*dist_vector
        force_attr_magnitude = ka*math.sqrt(dx*dx+dy*dy)

        # i think we should try to write a code as such we don't have to write a loop inside a function which is in callback so that loop does not interfere with ros
        '''lidar_data_filtered = []  
        for r in self.lidar_data.ranges:

            if r == float('inf') or r==float('nan'):
                continue
            else:
                lidar_data_filtered.append(r)'''
        


        print ("Distance Error = ", distance_error)
        #print ("Direction - Yaw = ",)     

 
        #print ("theta_obstacle = ", theta_obstacle)


        # writing Frepulsive 
        Frep_radius = 2

        Frep_radius = 2.0
        force_rep_x = 0.0
        force_rep_y = 0.0


        if len(obstacles_closest_point) == 0:
            force_reps_Vector = np.array([[0.0],[0.0]])
            force_reps_magnitude = 0.0

        else:
            force_reps_magnitude = 0.0
            force_rep_x = 0.0
            force_rep_y = 0.0
            for obs in obstacles_closest_point:
                x_obs, y_obs, distance_obs, theta_obs = obs 

                if distance_obs > Frep_radius:
                    continue # skips to the next 'i'

                force_reps_magnitude = kr * (1.0 / distance_obs - 1.0 / Frep_radius) / (distance_obs ** 2)

                # convert LiDAR frame → world frame
                angle_world = theta_obs + yaw

                force_rep_x += -force_reps_magnitude * math.cos(angle_world)
                force_rep_y += -force_reps_magnitude * math.sin(angle_world)

            force_reps_Vector = np.array([[force_rep_x],
                                        [force_rep_y]])



            '''if angle_error >= 0: # for 1st and 2nd quadrant i.e coordinates like (5,6) and (-5,6) worked
                                # issue for [-6,0] and so
                if angle_error <= 0.1:
                    
                    if distance <0.1:
                        msg.linear.x = 0.0 #write 0.0 not 0
                        msg.angular.z = 0.0              
                    else:    
                        msg.linear.x = 0.3
                
                else:
                    msg.angular.z = 0.3

            
            elif angle_error < 0:
                angle_error = -1*angle_error
                if angle_error <= 0.1:
                    
                    if distance <0.1:
                        msg.linear.x = 0.0  #write 0.0 not 0
                    else:    
                        msg.linear.x = 0.3
                
                else:
                    msg.angular.z = -0.3
            # LET'S MAKE A PID CONTROLLER INSTED OF DIS
            # if i make a proportional controller it will not be required to check for if statements to give CW and ACW angular velocity
            
            kp = 0.4
            kp2 = 0.1
            
            #kd = 0.001
            #dt = 0.001
            #prvs_distance_error = 0
            #prvs_distance_error = distance
            #time.sleep(dt)

            #differential_error_distance = (distance - prvs_distance_error)/dt

            vel_pub_x = kp*distance  # + kd*differential_error_distance'''


               

        print( " repulsive force magnitude = " ,force_reps_magnitude)
        print (" attractive force magnitude = ", force_attr_magnitude)
        force_resultant = force_reps_Vector + force_attr_vector
        print( "force resultant magnitude = ", force_resultant)

        desired_yaw = math.atan2(force_resultant[1,0], force_resultant[0,0])
        yaw_error = desired_yaw - yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        speed = math.sqrt(force_resultant[0,0]**2 + force_resultant[1,0]**2)

        print ("speed or F_resultant = ", speed)
        

        msg.linear.x = min(0.3, speed)
        
        msg.angular.z = 0.5*yaw_error

        self.command_vel_pub.publish(msg)






def main(args=None):    
    rclpy.init(args=args)

    controller_node = ControllerNode()


    rclpy.spin(controller_node)


    controller_node.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
 


