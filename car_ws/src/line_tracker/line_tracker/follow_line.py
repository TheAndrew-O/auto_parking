

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class FollowLine(Node):

    def __init__(self):
        super().__init__('follow_line')
        self.subscription = self.create_subscription(String, '/turn_left', self.turn_left_callback, 10)
        self.subscription = self.create_subscription(Point,'/detected_lines',self.listener_callback,10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.steer_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory',10)
        
        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.1)
        self.declare_parameter("forward_chase_speed", 0.4)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("max_size_thresh", 0.458)
        self.declare_parameter("filter_value", 0.9)


        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value


        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.counter = 0
        self.lastrcvtime = time.time() - 10000
        self.turn = False

    def timer_callback(self):
        msg = Twist()
        steer = JointTrajectory()
        steer.joint_names = ['front_left_steer_joint', 'front_right_steer_joint']
        
        point = JointTrajectoryPoint()
        #point.positions=[0.8,0.8]
        #point.time_from_start.sec = 1
        
        #steer.points.append(point)
        #self.steer_pub.publish(steer)

        if(self.turn == False):
            if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
                self.get_logger().info('Target: {}'.format(self.target_val))
                print("dist:",self.target_dist)
                if (self.target_dist < self.max_size_thresh):
                    print("forward")
                    self.counter = self.counter + 1
                    msg.linear.x = 0.32
                #msg.angular.z = -self.angular_chase_multiplier*self.target_val
                center_point = -self.angular_chase_multiplier*self.target_val
                point.positions = [center_point, center_point]
                point.time_from_start.sec = 1
                steer.points.append(point)
                self.steer_pub.publish(steer)
            else:
                self.get_logger().info('Target lost')
                msg.linear.x = self.forward_chase_speed
                point.positions=[0.0,0.0]
                point.time_from_start.sec = 1
                steer.points.append(point)
                self.steer_pub.publish(steer)
            self.publisher_.publish(msg)
        else:
            pass
    
    def turn_left_callback(self, msg):
        if(self.counter < 5):
            turn = msg.data
            cmd = Twist()
            steer = JointTrajectory()
            steer.joint_names = ['front_left_steer_joint', 'front_right_steer_joint']
            point = JointTrajectoryPoint()
            forward = [0.0,0.0]
            if(msg.data == 'left'):
                self.turn = True
                print("see left")
                cmd.linear.x = 0.35
                point.positions=[0.8727,0.8727]
                point.time_from_start.sec = 1
                steer.points.append(point)
                self.steer_pub.publish(steer)
                #cmd.linear.x = 0.08
            elif(msg.data == 'noturn'):
                print("no left")
                self.turn = False
                cmd.linear.x = 0.32
                point.positions = forward
                point.time_from_start.sec = 1
                steer.points.append(point)
                #self.steer_pub.publish(steer)
            else:
                self.turn = False
                cmd.linear.x = 0.32
                point.positions = forward
                point.time_from_start.sec = 1
                steer.points.append(point)
                #self.steer_pub.publish(steer)
            self.publisher_.publish(cmd)


    def listener_callback(self, msg):
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x
        self.target_dist = self.target_dist * f + msg.z
        self.lastrcvtime = time.time()
        # self.get_logger().info('Received: {} {}'.format(msg.x, msg.y))


def main(args=None):
    rclpy.init(args=args)
    follow_lines = FollowLine()
    rclpy.spin(follow_lines)
    follow_lines.destroy_node()
    rclpy.shutdown()
