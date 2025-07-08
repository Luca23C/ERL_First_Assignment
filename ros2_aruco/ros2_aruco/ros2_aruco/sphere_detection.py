import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String 


class SphereNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('sphere_detection')

        self.color_list = ['blue', 'red', 'black', 'green', 'yellow']
        self.counter = 0   # useful for counting the color in the list

        # Color mask definition
        # Red color
        self.red_lower1 = np.array([0, 120, 70], np.uint8)
        self.red_upper1 = np.array([10, 255, 255], np.uint8)
        self.red_lower2 = np.array([170, 120, 70], np.uint8)
        self.red_upper2 = np.array([180, 255, 255], np.uint8)

        # Green color
        self.green_lower = np.array([25, 52, 72], np.uint8)
        self.green_upper = np.array([102, 255, 255], np.uint8)

        # Blue color
        self.blue_lower = np.array([94, 80, 2], np.uint8)
        self.blue_upper = np.array([120, 255, 255], np.uint8)

        # Black color
        self.black_lower = np.array([0, 0, 0], np.uint8)
        self.black_upper = np.array([180, 255, 30], np.uint8)

        # Yellow color
        self.yellow_lower = np.array([15, 150, 150], np.uint8)
        self.yellow_upper = np.array([35, 255, 255], np.uint8)

        # Declare parameters
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value

        # Subscribers
        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)

        # Publisher
        self.publisher_image_on_topic = self.create_publisher(Image, '/output/image/rect', 10)
        self.timer = self.create_timer(0.5, self.img_pub_callback)
        self.publisher_status = self.create_publisher(String, '/status', 10)

        # Camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # Bridge for OpenCV
        self.bridge = CvBridge()

        # Image to publish
        self.send_img_topic = False
        self.image_to_pub_on_topic = None


    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        self.destroy_subscription(self.info_sub)


    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # Transform image in HSV
        hsvFrame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Red mask
        mask1 = cv2.inRange(hsvFrame, self.red_lower1, self.red_upper1)
        mask2 = cv2.inRange(hsvFrame, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Green mask
        mask_green = cv2.inRange(hsvFrame, self.green_lower, self.green_upper)

        # Blue mask     
        mask_blue = cv2.inRange(hsvFrame, self.blue_lower, self.blue_upper)

        # Black mask
        mask_black = cv2.inRange(hsvFrame, self.black_lower, self.black_upper)

        # Yellow mask
        mask_yellow = cv2.inRange(hsvFrame, self.yellow_lower, self.yellow_upper)

        # Dilatation
        kernel = np.ones((5, 5), "uint8")
        red_mask = cv2.dilate(red_mask, kernel)
        green_mask = cv2.dilate(mask_green, kernel)
        blue_mask = cv2.dilate(mask_blue, kernel)
        black_mask = cv2.dilate(mask_black, kernel)
        yellow_mask = cv2.dilate(mask_yellow, kernel)

        mask_list = [red_mask, green_mask, blue_mask, black_mask, yellow_mask] 
        mask_list_color = ['red', 'green', 'blue', 'black', 'yellow']

        # Find contourns
        for i in range(len(mask_list)):
            mask = mask_list[i]
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) != 0:
                mask_idx = i
                break
        
        # Check counter color list
        if self.counter >= len(self.color_list):
            status = String()
            status.data = 'Done'
            self.publisher_status.publish(status)
            self.counter = 0
        
        # Draw rect
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 300:
                x, y, w, h = cv2.boundingRect(contour)
                self.image_to_pub_on_topic = cv2.rectangle(cv_image.copy(), (x, y), (x + w, y + h), (0, 0, 255), 2)
                self.get_logger().info('Object found')

                color = self.color_list[self.counter]
                compare_color = mask_list_color[mask_idx]
                if color == compare_color:
                    self.counter = self.counter + 1
                    self.send_img_topic = True


    def img_pub_callback(self):
        if self.send_img_topic and self.image_to_pub_on_topic is not None:
            img_msg = self.bridge.cv2_to_imgmsg(self.image_to_pub_on_topic, encoding="bgr8")
            self.publisher_image_on_topic.publish(img_msg)
            self.send_img_topic = False


def main():
    rclpy.init()
    node = SphereNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()