"""
This node locates Aruco AR markers in images and publishes their ids and poses.
"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from ros2_aruco import transformations

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers


class ArucoNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('aruco_node')

        # Useful sharing variables
        self.x_coords = []
        self.y_coords = []
        self.lowest_id = 0
        self.start = True
        self.id_marker = 0
        self.number_of_aruco_in_world = 5
        self.marker_list = []
        self.start_pub_image = False
        self.count_list = 0
        self.create_list = True
        self.send_img_topic = False

        # Declare and read parameters
        self.declare_parameter("marker_size", .0625)
        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", None)

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(f"bad aruco_dictionary_id: {dictionary_id_name}")
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error(f"valid options: {options}")

        # Set up subscriptions
        self.info_sub = self.create_subscription(CameraInfo,
                                                 info_topic,
                                                 self.info_callback,
                                                 qos_profile_sensor_data)
        self.create_subscription(Image, image_topic,
                                 self.image_callback, qos_profile_sensor_data)

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        # Camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        self.bridge = CvBridge()

        # Additional pub/sub
        self.subscription = self.create_subscription(ArucoMarkers, '/aruco_markers', self.listener_id_aruco, 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_image, 10)

        self.publisher_image_on_topic = self.create_publisher(Image, '/output/image/circle', 10)
        self.timer = self.create_timer(0.5, self.img_pub_callback)
        self.publisher_status = self.create_publisher(String, '/status', 10)


    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        self.destroy_subscription(self.info_sub)


    def estimate_pose_single_markers(self, corners):
        rvecs = []
        tvecs = []
        for corner in corners:
            image_points = corner[0]
            object_points = np.array([
                [-self.marker_size/2,  self.marker_size/2, 0],
                [ self.marker_size/2,  self.marker_size/2, 0],
                [ self.marker_size/2, -self.marker_size/2, 0],
                [-self.marker_size/2, -self.marker_size/2, 0]
            ], dtype=np.float32)
            success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.intrinsic_mat, self.distortion)
            if success:
                rvecs.append(rvec)
                tvecs.append(tvec)
        return rvecs, tvecs


    def image_callback(self, img_msg):
        global marker_ids

        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
        markers = ArucoMarkers()
        pose_array = PoseArray()
        frame_id = self.camera_frame if self.camera_frame else self.info_msg.header.frame_id
        markers.header.frame_id = frame_id
        pose_array.header.frame_id = frame_id
        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = self.aruco_detector.detectMarkers(cv_image)

        if corners:
            array_3d_corner = corners[0]
            self.x_coords, self.y_coords = array_3d_corner[0][:, 0], array_3d_corner[0][:, 1]

        if marker_ids is not None:
            if self.create_list:
                if self.start:
                    self.id_marker = marker_ids[0, 0]
                    self.marker_list.append(self.id_marker)
                    self.start = False
                if marker_ids[0, 0] != self.id_marker and not self.start_pub_image:
                    self.id_marker = marker_ids[0, 0]
                    self.marker_list.append(marker_ids[0, 0])
                if len(self.marker_list) == self.number_of_aruco_in_world:
                    self.start_pub_image = True
                    self.lowest_id = min(self.marker_list)
                    self.marker_list = sorted(self.marker_list)
                    print(self.marker_list)
                    self.create_list = False

            rvecs, tvecs = self.estimate_pose_single_markers(corners)

            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pos = tvecs[i].flatten()
                pose.position.x = pos[0]
                pose.position.y = pos[1]
                pose.position.z = pos[2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(rvecs[i])[0]
                quat = transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)


    def listener_id_aruco(self, msg):
        pass


    def img_pub_callback(self):
        global image_to_pub_on_topic
        if self.send_img_topic:
            img_msg = self.bridge.cv2_to_imgmsg(image_to_pub_on_topic, encoding="bgr8")
            self.publisher_image_on_topic.publish(img_msg)
            self.send_img_topic = False


    def listener_image(self, msg):
        global image_to_pub_on_topic, marker_ids
        if self.start_pub_image and marker_ids is not None:
            if marker_ids[0, 0] == self.marker_list[self.count_list] and self.count_list < len(self.marker_list):
                print('marker: '+str(marker_ids[0,0]) + ' count: '+str(self.count_list))
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                sum_x, sum_y = sum(self.x_coords), sum(self.y_coords)
                x_center = round(sum_x/4)
                y_center = round(sum_y/4)
                center_array = np.array([x_center, y_center])

                if self.x_coords != [] and marker_ids is not None:
                    vertix = np.array([round(self.x_coords[0]), round(self.y_coords[0])])
                    radious = np.linalg.norm(center_array - vertix)
                    cv2.putText(cv_image, str(marker_ids[0, 0]), (x_center, y_center),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), thickness=2)
                    image_to_pub_on_topic = cv2.circle(cv_image, (x_center, y_center),
                                                       round(radious), (0,0,255), thickness=2)
                    self.count_list += 1
                    self.send_img_topic = True

                if self.count_list == len(self.marker_list):
                    self.start_pub_image = False
                    status = String()
                    status.data = 'Done'
                    self.publisher_status.publish(status)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

