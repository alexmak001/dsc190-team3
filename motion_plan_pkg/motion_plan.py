import rclpy
# import the ROS2 python libraries
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class MotionPlan(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('motion_plan')
        # create the publisher object (Output to Race Control)
            #self.publisher_ = self.create_publisher(msg/interface type (Twist, int, string, etc.),
            #                                        topic being published to ('cmd_vel'),
            #                                        queue size (10 usually, prevents late messages from stalling subscriber))
        self.publisher_ = self.create_publisher('OutputMsgType-TBD', 'INSERT TOPIC', 10)
        # create subsciber objects
            #self.subscriber = self.create_subscription(msg type (LaserScan, Twist, int, etc),
            #                                           topic being subscribed to ('/scan'),
            #                                           callback function (called whenever msg update),
            #                                           queue size (QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)))
        self.subscriber1 = self.create_subscription(msgType1, topic1, self.sensor_fusion, 10) #Sensor Fusion
        self.subscriber2 = self.create_subscription(msgType2, topic2, self.race_line, 10) #Race line
        self.subscriber3 = self.create_subscription(msgType3, topic3, self.behavior, 10) #Behavior plan
        # prevent unused variable warning

        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
            #self.variables = 0

        # create a Twist message, create output (make a path variable)
        self.msg = 'OutputMsgType-TBD'
        #self.timer = self.create_timer(timer period, callback function called every timer period) "updater"
        self.timer = self.create_timer(self.timer_period, self.plan_path)

    def sensor_fusion(self, msg):
        self.obstacle_pos = msg

    def race_line(self, msg): #import offline graph calculation and store here?
        self.race_line = msg

    def behavior(self, msg):
        self.behavior = msg

    def plan_path(self):
        #calculate path

        #publish to topic for Race Control Team
        self.publisher_.publish(self.msg)



def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    motion_plan = MotionPlan()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(motion_plan)
    # Explicity destroy the node
    motion_plan.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
