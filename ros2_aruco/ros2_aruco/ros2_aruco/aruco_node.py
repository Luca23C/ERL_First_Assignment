"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

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
        self.id_marker = 0 # variabile di appoggio iniziale
        self.number_of_aruco_in_world = 5
        self.marker_list = []  # list of all marker
        self.start_pub_image = False    # start to pub image on ros topic
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
        dictionary_id_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

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

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()


        # Adding pub/sub made by me
        # Subscriber for getting the id of the aruco
        self.subscription = self.create_subscription(ArucoMarkers, '/aruco_markers', self.listener_id_aruco, 10)
        self.subscription  # prevent unused variable warning

        # Subscriber for camera image
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_image, 10)
        self.subscription  # prevent unused variable warning

        # Publisher for publish image on custom topic
        self.publisher_image_on_topic = self.create_publisher(Image, '/output/image/circle', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.img_pub_callback)

        self.publisher_status = self.create_publisher(String, '/status', 10)


    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        global marker_ids

        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg,
                                             desired_encoding='mono8')
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame is None:
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame
            
            
        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image,
                                                                self.aruco_dictionary,
                                                                parameters=self.aruco_parameters)

        # Extract vertix of aruco marker
        if corners != ():
            array_3d_corner = corners[0]
            self.x_coords, self.y_coords = array_3d_corner[0][:, 0], array_3d_corner[0][:, 1]

        if marker_ids is not None:
            
            # Create list of aruco in the world
            if self.create_list:
                if self.start:
                    self.id_marker = marker_ids[0, 0]
                    self.marker_list.append(self.id_marker)
                    self.start = False
                
                if marker_ids[0, 0] != self.id_marker and self.start_pub_image == False:
                    self.id_marker = marker_ids[0, 0]
                    self.marker_list.append(marker_ids[0, 0])

                if len(self.marker_list) == self.number_of_aruco_in_world:
                    self.start_pub_image = True

                    self.lowest_id = self.marker_list[0]
                    for i in range(self.number_of_aruco_in_world):
                        aruco_id = self.marker_list[i]
                        if aruco_id < self.lowest_id:
                            self.lowest_id = aruco_id

                    # Order ids in the list 
                    self.marker_list = sorted(self.marker_list)
                    print(self.marker_list)
                    self.create_list = False


            if cv2.__version__ > '4.0.0':
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners,
                                                                      self.marker_size, self.intrinsic_mat,
                                                                      self.distortion)
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners,
                                                                   self.marker_size, self.intrinsic_mat,
                                                                   self.distortion)
            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
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

    
    # Callback made by me
    def listener_id_aruco(self, msg):
        # Getting id of aruco
        marker_ids = msg.marker_ids
        #self.get_logger().info(f"Detected marker IDs: {marker_ids}")


    def img_pub_callback(self):
        # Publish image on custom topic
        if self.send_img_topic:
            img_msg = self.bridge.cv2_to_imgmsg(image_to_pub_on_topic, encoding="bgr8")
            self.publisher_image_on_topic.publish(img_msg)
            self.send_img_topic = False
            

    def listener_image(self, msg):
        global image_to_pub_on_topic
        # Subscribe the image raw and convert into opencv format
        if self.start_pub_image == True and marker_ids is not None:
            if marker_ids[0, 0] == self.marker_list[self.count_list] and self.count_list < len(self.marker_list):
                print('marker: '+str(marker_ids[0,0]) + 'count: '+str(self.count_list))
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                sum_x = 0
                sum_y = 0
                # Get information about the coordinate of the marker
                for i in range(len(self.x_coords)):
                    sum_x = sum_x + self.x_coords[i]
                    sum_y = sum_y + self.y_coords[i]
                # Coordinate of the center of the marker
                x_center = round(sum_x/4)
                y_center = round(sum_y/4)
                center_array = np.array([x_center, y_center])
                
                if self.x_coords != [] and marker_ids is not None:
                    vertix = np.array([round(self.x_coords[0]), round(self.y_coords[0])])
                    radious = np.linalg.norm(center_array - vertix)
                    cv2.putText(cv_image, str(marker_ids[0, 0]), (x_center, y_center), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), thickness=2)
                    image_to_pub_on_topic = cv2.circle(cv_image, (x_center, y_center), round(radious), (0,0,255), thickness=2)
                    self.count_list += 1
                    self.send_img_topic = True
                
                # When all marker are published
                if self.count_list == len(self.marker_list):
                    self.start_pub_image = False
                    status = String()
                    status.data = 'Done'
                    self.publisher_status.publish(status)

                # If you uncomment this two lines you can see the tracking of the aruco
                #cv2.imshow('hello', cv_image)
                #cv2.waitKey(1)
        

def main():
    rclpy.init()
    node = ArucoNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
