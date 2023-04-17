import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import line_tracker.image_process as proc

class DetectLine(Node):
    def __init__(self):
        super().__init__('detect_line')
        self.get_logger().info('Looking for parking lane')
        self.image_sub = self.create_subscription(Image,"/front_camera_sensor/image_raw",self.callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.left_image = self.create_subscription(Image,"/left_camera_sensor/image_raw",self.turn_callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        #self.right_image = self.create_subscription(Image,"/right_camera_sensor/image_raw",self.callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_pub = self.create_publisher(Image, "/image_tuning", 1)
        self.left_image_out = self.create_publisher(Image, "/left_image_out", 1)
        self.line_pub  = self.create_publisher(Point,"/detected_lines",1)
        self.turn_left_pub = self.create_publisher(String, "/turn_left", 1)
        self.declare_parameter('tuning_mode', False)

        self.declare_parameter("x_min",0)
        self.declare_parameter("x_max",100)
        self.declare_parameter("y_min",35)
        self.declare_parameter("y_max",100)
        self.declare_parameter("h_min",110)
        self.declare_parameter("h_max",157)
        self.declare_parameter("s_min",85)
        self.declare_parameter("s_max",255)
        self.declare_parameter("v_min",0)
        self.declare_parameter("v_max",255)
        self.declare_parameter("sz_min",0)
        self.declare_parameter("sz_max",100)
        self.declare_parameter("x_min2",15)
        self.declare_parameter("x_max2",90)
        self.declare_parameter("y_min2",35)
        self.declare_parameter("y_max2",100)
        self.declare_parameter("h_min2",117)
        self.declare_parameter("h_max2",128)
        self.declare_parameter("s_min2",105)
        self.declare_parameter("s_max2",255)
        self.declare_parameter("v_min2",0)
        self.declare_parameter("v_max2",255)
        self.declare_parameter("sz_min2",0)
        self.declare_parameter("sz_max2",100)


        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.tuning_params = {
            'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().integer_value,
            'h_min': self.get_parameter('h_min').get_parameter_value().integer_value,
            'h_max': self.get_parameter('h_max').get_parameter_value().integer_value,
            's_min': self.get_parameter('s_min').get_parameter_value().integer_value,
            's_max': self.get_parameter('s_max').get_parameter_value().integer_value,
            'v_min': self.get_parameter('v_min').get_parameter_value().integer_value,
            'v_max': self.get_parameter('v_max').get_parameter_value().integer_value,
            'sz_min': self.get_parameter('sz_min').get_parameter_value().integer_value,
            'sz_max': self.get_parameter('sz_max').get_parameter_value().integer_value
        }
        self.tuning_params2 = {
            'x_min': self.get_parameter('x_min2').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max2').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min2').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max2').get_parameter_value().integer_value,
            'h_min': self.get_parameter('h_min2').get_parameter_value().integer_value,
            'h_max': self.get_parameter('h_max2').get_parameter_value().integer_value,
            's_min': self.get_parameter('s_min2').get_parameter_value().integer_value,
            's_max': self.get_parameter('s_max2').get_parameter_value().integer_value,
            'v_min': self.get_parameter('v_min2').get_parameter_value().integer_value,
            'v_max': self.get_parameter('v_max2').get_parameter_value().integer_value,
            'sz_min': self.get_parameter('sz_min2').get_parameter_value().integer_value,
            'sz_max': self.get_parameter('sz_max2').get_parameter_value().integer_value
        }

        self.bridge = CvBridge()

        if(self.tuning_mode):
            proc.create_tuning_window(self.tuning_params)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            if (self.tuning_mode):
                self.tuning_params = proc.get_tuning_params()
            lines, out_image, tuning_image = proc.find_lines(cv_image, self.tuning_params)
            
            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)
            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = data.header
            self.image_tuning_pub.publish(img_to_pub)
            point = Point()
            if lines is not None:
                for i, line in enumerate(lines):
                    x = line.pt[0]
                    y = line.pt[1]
                    size = line.size
                    self.get_logger().info(f"Pt: ({x},{y},{size})")
                    if(size > point.z):
                        point.x = float(x)
                        point.y = float(y)
                        point.z = float(size)
                if(point.z > 0):
                    self.line_pub.publish(point)
        except CvBridgeError as e:
            print(e)

    def turn_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            if (self.tuning_mode):
                self.tuning_params = proc.get_tuning_params()
            lines, out_image, tuning_image = proc.find_lines(cv_image, self.tuning_params2)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.left_image_out.publish(img_to_pub)
            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = data.header
            #self.image_tuning_pub.publish(img_to_pub)
            point = Point()
            if lines is not None:
                for i, line in enumerate(lines):
                    x = line.pt[0]
                    y = line.pt[1]
                    size = line.size
                    self.get_logger().info(f"Pt: ({x},{y},{size})")
                    if(size > point.z):
                        point.x = float(x)
                        point.y = float(y)
                        point.z = float(size)
                msg = String()
                if(point.z > 0):
                    msg.data = 'left'
                    self.turn_left_pub.publish(msg)
                else:
                    msg.data = 'noturn'
                    self.turn_left_pub.publish(msg)
              

        except CvBridgeError as e:
            print(e)

def main(args=None):

    rclpy.init(args=args)

    detect_line = DetectLine()
    while rclpy.ok():
        rclpy.spin_once(detect_line)
        proc.wait_on_gui()

    detect_line.destroy_node()
    rclpy.shutdown()

