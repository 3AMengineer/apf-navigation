import rclpy
from rclpy.node import Node # node is a module inside rclpy and Node is a class inside rclpy.node
from std_msgs.msg import String
#These lines represent the node’s dependencies. Recall that dependencies have to be added to package.xml

class OdomSubscriber(Node): # inherited OdomSubscriber from Node class
    
    def __init__(self):
        super().__init__('odom_subscriber') #Following is the definition of the class’s constructor. super().__init__ calls the Node class’s constructor and gives it your node name, in this case odom_publisher.
        
        self.subscription = self.create_subscription(String,'odom',self.listener_callback,10)
        #The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message.

    def listener_callback(self, odom_data):
        self.get_logger().info(odom_data.data)

    def main(args=None):
        rclpy.init(arg=args)

        odom_subscriber = OdomSubscriber()

        rclpy.spin(odom_subscriber)

        odom_subscriber.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()





