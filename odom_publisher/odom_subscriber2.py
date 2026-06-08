import rclpy
from rclpy.node import Node # node is a module inside rclpy and Node is a class inside rclpy.node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import math



#These lines represent the node’s dependencies. Recall that dependencies have to be added to package.xml

class OdomSubscriber(Node): # inherited OdomSubscriber from Node class
    
    def __init__(self):
        super().__init__('odom_subscriber') #Following is the definition of the class’s constructor. super().__init__ calls the Node class’s constructor and gives it your node name, in this case odom_publisher.
        
        self.subscription_odom = self.create_subscription(Odometry,'/odom',self.listener_callback_odom,10) # create_subscription(Data_type,topicName,QoS)

        self.subscription_lidar = self.create_subscription(LaserScan,'/scan',self.listener_callback_lidar,10)

        self.controller_timer = self.create_timer(0.1 , self.apfa_controller)
        # We need to add a create_timer function to call the apfa_controller function periodically
        
        self.command_vel_pub =  self.create_publisher(Twist,'cmd_vel',10)
        # the function create_publisher returns an object that object is equated to the variable command_vel_pub

        #The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message.

    def listener_callback_odom(self, odom_data):
        # self.get_logger().info(odom_data.twist)   #get_logger().info is only for string.
        
        self.odom_data = odom_data
    

    def listener_callback_lidar(self,lidar_data):

        self.lidar_data = lidar_data

        
    def apfa_controller(self):
    
        x_odom = self.odom_data.pose.pose.position.x
        y_odom = self.odom_data.pose.pose.position.y
        z_odom = self.odom_data.pose.pose.position.z

        quat_x = self.odom_data.pose.pose.orientation.x
        quat_y = self.odom_data.pose.pose.orientation.y
        quat_z = self.odom_data.pose.pose.orientation.z
        quat_w = self.odom_data.pose.pose.orientation.w

        velocity_x = self.odom_data.twist.twist.linear.x

        ang_vel_z = self.odom_data.twist.twist.angular.z

        quaternion = [quat_x,quat_y, quat_z, quat_w]  # Replace with actual values
        
        yaw = math.atan2(2.0 * (quat_w * quat_z + quat_x * quat_y),1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z))

        lidar_data_filtered = []  


        for r in self.lidar_data.ranges:

            if r == float('inf') or r==float('nan'):
                continue
            else:
                lidar_data_filtered.append(r)
        msg = Twist()
        
        goal_position  = [0,6]
        current_position = [x_odom,y_odom]
        
        dx = goal_position[0] - x_odom
        dy = goal_position[1] - y_odom

        distance = math.dist(goal_position,current_position)
        direction = math.atan2(dy,dx)
        angle_error = direction-yaw
        # Normalize angle error to [-pi, pi] so the robot takes the shortest turn
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if distance < 0.2:
            msg = Twist()
            msg.angular.z = 0.0
            self.command_vel_pub.publish(msg)
            return        
        



                
        print ("distance=",distance)
        print ("direction - yaw",angle_error)        
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
                msg.angular.z = -0.3'''
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

        vel_pub_x = kp*distance  # + kd*differential_error_distance 
        w_pub_z = kp*angle_error 
        
        if angle_error <=0.1:
            msg.linear.x = kp*distance
            msg.angular.z = kp2*angle_error
        else:
            msg.linear.x = kp2*distance
            msg.angular.z = kp*angle_error

        self.command_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    odom_subscriber = OdomSubscriber()

    rclpy.spin(odom_subscriber)


    odom_subscriber.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
 


