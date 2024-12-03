import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray



class ControlCamera(Node):

    def __init__(self):
        super().__init__('control_camera')
        # Publish conrol rotation for camera
        self.pub_camera_control = self.create_publisher(Float64MultiArray, '/camera_joint_z_axis_controller/commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback_camera_control)
        self.angle = 0.0
        self.start = True

    def timer_callback_camera_control(self):
        # Routine for controlling camera
        control_msg = Float64MultiArray()
        if self.start:
            self.angle = self.angle + 0.1
            self.start = False
        else:
            self.angle = self.angle + 0.1
        
        if self.angle > 6.28:
            self.angle = 0.0
            self.start = True

        control_msg.data = [self.angle]
        self.pub_camera_control.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)

    control = ControlCamera()

    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()