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
        self.odom_data_time = time.time()
    
    def listener_callback_lidar(self,lidar_data):
        self.lidar_data = lidar_data
        self.lidar_data_time = time.time()
        
    def apfa_controller(self):
        # Convert LiDAR range list to a NumPy array
        lidar_ranges = np.array(self.lidar_data.ranges)
        angle_min = self.lidar_data.angle_min
        angle_increment = self.lidar_data.angle_increment
        #print(lidar_ranges)

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

        ka = 0.6
        kr = 10

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
        
        mask = np.isfinite(lidar_ranges) # gives Boolean True False 
        clean_lidar_data = lidar_ranges[mask]   

        angles_array = (self.lidar_data.angle_min + np.arange(lidar_ranges.shape[0]) * self.lidar_data.angle_increment)
        
        # lidar_ranges.shape[0] gives us the 0th index of the shape of the array. i.e for [1,2,3] it gives 3
        # so basically we are creating an array of length as per the lidar_ranges
        # angle_array now stores all the data of angles startong from angle_min to the highest index in ranges 
        # Now let's add the mask we added to lidar_data to angles_array also

        clean_angles_array = angles_array[mask]

        print ("lidar_data = ", clean_lidar_data)      

        print ("Distance Error = ", distance_error)
        #print ("Direction - Yaw = ",)     

 
        #print ("theta_obstacle = ", theta_obstacle)


        # writing Frepulsive 
        Frep_radius = 2


        if clean_lidar_data.size == 0:
            force_reps_Vector = np.array([[0.0],[0.0]])
            force_reps_magnitude = 0.0

        else:
            min_index = np.argmin(clean_lidar_data) #returns the index of the minimum value within that array, not the original array.
            distance_obstacle = clean_lidar_data[min_index]
            theta_obstacle = yaw + clean_angles_array[min_index]  

            if distance_obstacle <= Frep_radius :
                force_reps_magnitude = kr *(1.0/distance_obstacle - 1.0/Frep_radius) * (1.0/(distance_obstacle**3))
                if distance_obstacle <= 0.8 :
                    force_reps_magnitude = kr*kr*kr*(1.0/distance_obstacle - 1.0/Frep_radius) * (1.0/(distance_obstacle**3))

            else:
                force_reps_magnitude = 0.0

            force_reps_x = -force_reps_magnitude * np.cos(theta_obstacle)
            force_reps_y = -force_reps_magnitude * np.sin(theta_obstacle)

            force_reps_Vector = np.array([[force_reps_x],[force_reps_y]])



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
 


# WALL AND POLE LOOKS SAME FOR THIS CODE 
# APF works on closest distance of a cluster(object) not all of the world
# APF is not “nearest obstacle avoidance” — it is vector-field summation from many obstacles.