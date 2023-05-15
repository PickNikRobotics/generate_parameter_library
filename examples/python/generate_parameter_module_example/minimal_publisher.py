# -*- coding: utf-8 -*-
import rclpy
import rclpy.node

from admittance_parameters import admittance_controller


class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__("admittance_controller")
        self.timer = self.create_timer(1, self.timer_callback)

        self.param_listener = admittance_controller.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.get_logger().info(
            "Initial control frame parameter is: '%s'" % self.params.control.frame.id
        )
        self.get_logger().info(
            "fixed string is: '%s'" % self.params.fixed_string
        )

        self.get_logger().info(
            "Original joints parameter is: '%s'" % str(self.params.joints)
        )
        for d in self.params.fixed_array:
            self.get_logger().info("value: '%s'" % str(d))

    def timer_callback(self):
        if self.param_listener.is_old(self.params):
            self.params = self.param_listener.get_params()
            self.get_logger().info("New control frame parameter is: '%s'" % self.params.control.frame.id)
            self.get_logger().info("fixed string is: '%s'" % self.params.fixed_string)
            for d in self.params.fixed_array:
                self.get_logger().info("value: '%s'" % str(d))


def main(args=None):
    rclpy.init(args=args)
    node = MinimalParam()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
