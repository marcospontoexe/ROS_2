import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

class posePublisher(Node):

    def __init__(self):

        super().__init__('setPose')

        self.subscriber = self.create_subscription(PointStamped, '/clicked_point', self.pointCallback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.pubPoint = PoseWithCovarianceStamped()
        #self.timer = self.create_timer(0.5, self.pubPoint

    def pointCallback(self, msg):

        self.initialPub(msg.point.x, msg.point.y, msg.point.z)

    def initialPub(self, x, y, z):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = '/map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        self.get_logger().info('Publishing  Initial Position  \n X= %f \n Y= %f '% (msg.pose.pose.position.x, msg.pose.pose.position.y))
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    pose = posePublisher()
    rclpy.spin(pose)
    pose.destroy_node()
    rclpy.shutdown()


main()