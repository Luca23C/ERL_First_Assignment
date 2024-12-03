import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String



class ControlVel(Node):

    def __init__(self):
        super().__init__('control_robot_vel')
        # Publish conrol velocity of the robot
        self.pub_vel_control = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback_vel_control)
        self.status = True

        # Subscriber for stopping the rover when all the markers are detected
        self.subscription_status = self.create_subscription(String, '/status', self.listener_status, 10)


    def timer_callback_vel_control(self):
        # Routine for controlling robot velocity
        control_msg = Twist()
        control_msg.angular.z = 0.5
        if self.status:
            self.pub_vel_control.publish(control_msg)
        else:
            control_msg.angular.z = 0.0
            self.pub_vel_control.publish(control_msg)

    def listener_status(self, msg):
        status = msg.data
        if status == 'Done':
            self.status = False
        

def main(args=None):
    rclpy.init(args=args)

    control = ControlVel()

    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()