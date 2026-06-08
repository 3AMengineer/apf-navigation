import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray



class ControllerNode(Node):

    def __init__(self, xd, yd, ka, kr, ktheta, gstar, eps_orient, eps_control):

        super().__init__('apf_controller')

        self.xd = xd
        self.yd = yd
        self.ka = ka
        self.kr = kr
        self.ktheta = ktheta
        self.gstar = gstar
        self.eps_orient = eps_orient
        self.eps_control = eps_control

        self.OdometryMessage = Odometry()
        self.DepthcamMessage = PointCloud2()
        self.ControlVelocity = Twist()

        self.OdometrySubscriber = self.create_subscription(Odometry, '/odom', self.OdometryCallback, 10)
        self.DepthcamSubscriber = self.create_subscription(PointCloud2, '/depth_camera/points', self.DepthcamCallback, 10)
        self.ClusterPublisher = self.create_publisher(MarkerArray, "/apf_clusters", 10)
        self.VelocityPublisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.points = np.array([])
        self.timer = self.create_timer(0.05, self.ControlFunction)

    
    def OdometryCallback(self, recieved_message):
        self.OdometryMessage = recieved_message


    def DepthcamCallback(self, recieved_message):
        pts = point_cloud2.read_points_numpy(recieved_message, field_names=('x','y','z'), skip_nans=True)

        if pts.ndim == 2:
            self.points = pts
        else:
            self.points = np.vstack((pts['x'], pts['y'], pts['z'])).T

        height = self.points[:,2]
        self.points = self.points[(height > 0.08) & (height < 2.0)]


    def ObstacleClustering(self):

        clusters = []

        if len(self.points) > 0:
            xy = self.points[:, :2]

            eps = 0.2
            min_samples = 4
            db = DBSCAN(eps = eps, min_samples = min_samples)
            labels = db.fit_predict(xy)

            cluster_ids = set(labels)
            cluster_ids.discard(-1)
            # clusters = []
            
            for cid in cluster_ids:
                cluster_pts = (xy)[labels == cid]
                clusters.append(cluster_pts)

            return clusters
        
        else:
            return clusters
    
    def PublishClusters(self, clusters):

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, cluster in enumerate(clusters):

            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = now
            m.ns = "clusters"
            m.id = i
            m.type = Marker.POINTS
            m.action = Marker.ADD

            m.scale.x = 0.05
            m.scale.y = 0.05

            np.random.seed(i)
            m.color.r = np.random.rand()
            m.color.g = np.random.rand()
            m.color.b = np.random.rand()
            m.color.a = 1.0

            for p in cluster:
                pt = Point()
                pt.x = float(p[0])
                pt.y = float(p[1])
                pt.z = 0.0
                m.points.append(pt)

            marker_array.markers.append(m)

        self.ClusterPublisher.publish(marker_array)


    def AngleRotation(self, theta, thetad):

        if (thetad > np.pi/2) and (thetad <= np.pi):
            
            if (theta > -np.pi) and (theta <= -np.pi/2):
                theta=theta+2*np.pi
        
        if (theta > np.pi/2) and (theta <= np.pi):
            
            if (thetad > -np.pi) and (thetad <= -np.pi/2):
                thetad=thetad+2*np.pi
        
        errorOrientation=thetad-theta
        return errorOrientation
    

    def ControlFunction(self):

        x = self.OdometryMessage.pose.pose.position.x
        y = self.OdometryMessage.pose.pose.position.y

        quat = self.OdometryMessage.pose.pose.orientation
        # quatl = [quat.x, quat.y, quat.z, quat.w]
        # yaw = euler_from_quaternion(quatl)[2]

        yaw = np.arctan2(2*(quat.w*quat.z + quat.x*quat.y), 1 - 2*(quat.y**2 + quat.z**2))

        theta = yaw

        clusters = self.ObstacleClustering()
        self.PublishClusters(clusters)

        Fr = np.zeros(2)

        if clusters:
            for cluster in clusters:
                distances = np.linalg.norm(cluster, ord = 2, axis = 1)
                i_min = np.argmin(distances)
                closest = cluster[i_min]
                d = distances[i_min]

                if d < self.gstar:
                    magnitude = self.kr * (1/d - 1/self.gstar) * (1/d**2)
                    direction_robot = -closest / d

                    cos, sin = np.cos(theta), np.sin(theta)

                    rotation_matrix = np.array([[cos, -sin], [sin, cos]])

                    direction = rotation_matrix @ direction_robot

                    Fr += magnitude * direction

        else:
            Fr = 0

        Fa = self.ka * np.array([self.xd - x, self.yd - y])
        # c, s = np.cos(theta), np.sin(theta)
        # R = np.array([[ c,  s],
        #             [-s,  c]])   # world → robot
        # Fa = R @ Fa_world

        Fnet = Fa + Fr

        thetaD = np.arctan2(Fnet[1], Fnet[0])

        angle_rotate = self.AngleRotation(theta, thetaD)

        if np.linalg.norm([x-self.xd, y-self.yd]) < self.eps_control:
            v = 0.0
            w = 0.0
        
        else:
            if abs(angle_rotate) > self.eps_orient:
                v = 0.0
                w = self.ktheta * angle_rotate

            else:
                v = np.linalg.norm(Fnet)
                w = self.ktheta * angle_rotate

        self.ControlVelocity.linear.x = float(min(v, 2.5))
        self.ControlVelocity.angular.z = float(w)

        self.VelocityPublisher.publish(self.ControlVelocity)
        


def main(args = None):

    rclpy.init(args = args)

    xd = 10
    yd = 10

    ka = 1.0
    kr = 5.0
    ktheta = 0.5

    gstar = 1.0

    eps_orient = np.pi/10
    eps_control = 1

    controller_node = ControllerNode(xd, yd, ka, kr, ktheta, gstar, eps_orient, eps_control)

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()