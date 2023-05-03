import rclpy
import rclpy.node

from admittance_parameters import admittance_controller


class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('admittance_controller')
        self.timer = self.create_timer(1, self.timer_callback)

        self.param_listener = admittance_controller.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.get_logger().info("Initial control frame parameter is: '%s'" % self.params.control.frame.id)

        self.get_logger().info("Original joints parameter is: '%s'" % str(self.params.joints))

    def timer_callback(self):
        self.params = self.param_listener.get_params()
        self.get_logger().info("New joints parameter is: '%s'" % str(self.params.joints))


def main(args=None):
    rclpy.init(args=args)
    node = MinimalParam()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
