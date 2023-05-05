

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class FollowLine(Node):

    def __init__(self):
        super().__init__('follow_line')
        self.subscription = self.create_subscription(String, '/turn_left', self.turn_left_callback, 10)
        self.subscription = self.create_subscription(String, '/turn_right', self.turn_right_callback, 10)
        self.subscription = self.create_subscription(Point,'/detected_lines',self.listener_callback,10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.steer_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory',10)
        
        self.declare_parameter("wait_time_sec", 1.0)
        self.declare_parameter("turn_scalar", 0.1)
        self.declare_parameter("forward_speed", 0.4)
        self.declare_parameter("turn_vel", 0.5)
        self.declare_parameter("max_dist_threshold", 0.66)
        self.declare_parameter("vel_scalar", 0.9)


        self.wait_time_sec = self.get_parameter('wait_time_sec').get_parameter_value().double_value
        self.turn_scalar = self.get_parameter('turn_scalar').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.turn_vel = self.get_parameter('turn_vel').get_parameter_value().double_value
        self.max_dist_threshold = self.get_parameter('max_dist_threshold').get_parameter_value().double_value
        self.vel_scalar = self.get_parameter('vel_scalar').get_parameter_value().double_value

        self.front_min_distance = 0.0
        self.right_min_distance = 0.0
        self.lef_min_distance = 0.0
        self.min_dist_threshold = 0.0

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.counter = 0
        self.lastrcvtime = time.time() - 10000
        self.turn = False
        self.right = False
        self.left = False
        self.seeking = True
        self.adjusting = False

    def scan_callback(self, scan):
        self.right_min_distance = min(scan.ranges[0:139])
        self.front_min_distance = min(scan.ranges[140:219])
        self.left_min_distance = min(scan.ranges[220:359])
        cmd = Twist()
        steer = JointTrajectory()
        steer.joint_names = ['front_left_steer_joint', 'front_right_steer_joint']
        point = JointTrajectoryPoint()
        #print(self.front_min_distance)
        if(self.seeking == False):
            print("right:",self.right_min_distance, "left:",self.left_min_distance,"front:",self.front_min_distance)
            self.adjusting=True
            if(self.front_min_distance < 0.77):
                self.get_logger().info('Parked')
                self.destroy_node()
                rclpy.shutdown()
            elif(self.right_min_distance < 1.39):
                print("adjust left")
                turn_rate = self.right_min_distance + 0.25
                point.positions=[0.45,0.45]
                point.time_from_start.sec = 1
                cmd.linear.x = 0.22
                steer.points.append(point)
                self.steer_pub.publish(steer)
                self.publisher_.publish(cmd)
            elif(self.left_min_distance < 1.25):
                print("adjust right")
                turn_rate = -1 * (self.left_min_distance + 0.25)
                point.positions=[-0.45,-0.45]
                point.time_from_start.sec = 1
                cmd.linear.x = 0.22
                steer.points.append(point)
                self.steer_pub.publish(steer)
                self.publisher_.publish(cmd)
            else:
                self.adjusting = False

        

    def timer_callback(self):
        msg = Twist()
        steer = JointTrajectory()
        steer.joint_names = ['front_left_steer_joint', 'front_right_steer_joint']
        
        point = JointTrajectoryPoint()
        #point.positions=[0.8,0.8]
        #point.time_from_start.sec = 1
        
        #steer.points.append(point)
        #self.steer_pub.publish(steer)
        # FORWARD SEEKING CONDITION
        if(self.turn == False):
            if (time.time() - self.lastrcvtime < self.wait_time_sec):
                #self.get_logger().info('Target: {}'.format(self.target_val))
                #print("dist:",self.target_dist)
                self.seeking = False
                if (self.target_dist < self.max_dist_threshold):
                    #print("forward")
                    self.counter = self.counter + 1
                    msg.linear.x = 0.32
                #msg.angular.z = -self.turn_scalar*self.target_val
                if(self.adjusting == False):
                    print("seeking")
                    center_point = -self.turn_scalar*self.target_val
                    point.positions = [center_point, center_point]
                    point.time_from_start.sec = 1
                    steer.points.append(point)
                    self.steer_pub.publish(steer)
            else:
                #self.get_logger().info('Target lost')
                self.seeking = True
                msg.linear.x = self.forward_speed
                point.positions=[0.0,0.0]
                point.time_from_start.sec = 1
                steer.points.append(point)
                self.steer_pub.publish(steer)
            self.publisher_.publish(msg)
        else:
            pass
        
    # TODO: ADD CONDITION FOR IF msg.data == 'right' (right perpendicular parking) steer right pos = [-0.8727,-0.8727]
    # TODO: ADD CONDITION FOR IF msg.data == 'back' (reverse perpendicular parking)
    def turn_left_callback(self, msg):
        #ignore left_cam if front cam can see target
        if(self.counter < 5):
            turn = msg.data
            cmd = Twist()
            steer = JointTrajectory()
            steer.joint_names = ['front_left_steer_joint', 'front_right_steer_joint']
            point = JointTrajectoryPoint()
            forward = [0.0,0.0]
            if(msg.data == 'left'):
                self.turn = True
                self.left = True
                self.right = False
                print("see left")
                cmd.linear.x = 0.35
                point.positions=[0.8727,0.8727]
                point.time_from_start.sec = 1
                steer.points.append(point)
                self.steer_pub.publish(steer)
                self.publisher_.publish(cmd)
                #cmd.linear.x = 0.08
            elif(msg.data == 'noturn' and self.right == False):
                self.turn = False
                self.left = False
                self.right = False
            else:
                cmd.linear.x = 0.32

    def turn_right_callback(self, msg):
        #ignore left_cam if front cam can see target
        if(self.counter < 5):
            turn = msg.data
            cmd = Twist()
            steer = JointTrajectory()
            steer.joint_names = ['front_left_steer_joint', 'front_right_steer_joint']
            point = JointTrajectoryPoint()
            forward = [0.0,0.0]
            if(msg.data == 'right'):
                self.turn = True
                self.right = True
                self.left = False
                print("see right")
                cmd.linear.x = 0.35
                point.positions=[-0.8727,-0.8727]
                point.time_from_start.sec = 1
                steer.points.append(point)
                self.steer_pub.publish(steer)
                self.publisher_.publish(cmd)
                #cmd.linear.x = 0.08
            elif(msg.data == 'noturn' and self.left == False):
                self.turn = False
                self.left = False
                self.right = False
            else:
                cmd.linear.x = 0.32


    # Calc angle to goal point
    def listener_callback(self, msg):
        scalar = self.vel_scalar
        self.target_val = (self.target_val * scalar) + msg.x
        self.target_dist = (self.target_dist * scalar) + msg.z
        self.lastrcvtime = time.time()
        # self.get_logger().info('Received: {} {}'.format(msg.x, msg.y))


def main(args=None):
    rclpy.init(args=args)
    follow_lines = FollowLine()
    rclpy.spin(follow_lines)
    follow_lines.destroy_node()
    rclpy.shutdown()
