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
        self.timer_period = 0.02
        # define the variable to save the received info
        self.laser_front = 0    # distance to front
        self.laser_right_min = 0    # min distnace to right
        self.laser_right_max = 0  # max distance to right
        self.laser_left_min = 0 # min distance to left

        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def move_turtlebot(self, msg):
        # Save the right laser scan info front
        # front laser scan, find closest item
        self.laser_front_min = min(msg.ranges[405:408]) #not rlly needed, three degrees front

        # min distance to right and left laser scan
        self.laser_right_min = min(msg.ranges[132:141]) #range of three degrees on right
        self.laser_left_min = min(msg.ranges[672:681]) #range of three degrees on left



    def motion(self):
        # print the data
        self.get_logger().info("Laser Left Min: {0} \n Linear.X {1} \n Linear.Z = {2}".format(
            self.laser_left_min, self.cmd.linear.x, self.cmd.angular.z))
        # Logic of move

        if self.laser_left_min < 0.22: # too close to left wall
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.1

        elif self.laser_left_min > 0.28: #far from wall left wall
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.1

        else: # perfect
            self.cmd.linear.x = 0.7
            self.cmd.angular.z = 0.1

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
