import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile


class topicsp1(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('topicsp1')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.move_turtlebot, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        # self.subscriber
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_front = 0    # distance to front
        self.laser_right_min = 0    # min distnace to right
        self.laser_right_max = 0  # max distance to right

        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def move_turtlebot(self, msg):

        # Save the right laser scan info front
        # front laser scan, find closest item
        self.laser_front = min(msg.ranges[170:190])

        # min distnace to right laser scan
        self.laser_right_min = min(msg.ranges[70:110])
        # max distance to right
        self.laser_right_max = max(msg.ranges[70:110])

    def motion(self):
        # print the data
        self.get_logger().info("Laser Front: {0} \n Laser Right Min: {1} \n Laser Right Max: {2} \n Linear.X {3}".format(
            self.laser_front, self.laser_right_min, self.laser_right_max, self.cmd.linear.x))
        # Logic of move

        if self.laser_front < 0.5:  # incoming corner
            self.cmd.linear.x = 0.09
            self.cmd.angular.z = 0.4
        elif self.laser_front < 0.2:  # front too close to wall
            self.cmd.linear.x = 0.05
            self.cmd.angular.z = 1
        elif self.laser_right_max > 0.3:  # far from wall
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = -0.1
        elif self.laser_right_min < 0.2:  # too close to wall
            self.cmd.linear.x = 0.06
            self.cmd.angular.z = 0.1
        else:  # perfect place
            self.cmd.linear.x = 0.11
            self.cmd.angular.z = 0.0

        # Publishing the cmd_vel values to topipc
        self.publisher_.publish(self.cmd)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    topic1 = topicsp1()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(topic1)
    # Explicity destroy the node
    topic1.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
