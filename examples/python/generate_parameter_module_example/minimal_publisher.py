import rclpy
import rclpy.node

from admittance_parameters import admittance_controller


class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)


def main():
    rclpy.init()
    node = MinimalParam()

    paramListener = admittance_controller.ParamListener(node)
    params = paramListener.get_params()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
