# -*- coding: utf-8 -*-
# Copyright 2023 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
import rclpy.node

from cmake_generate_parameter_module_example.admittance_parameters import (
    admittance_controller,
)


class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('admittance_controller')
        self.timer = self.create_timer(1, self.timer_callback)

        self.param_listener = admittance_controller.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.get_logger().info(
            "Initial control frame parameter is: '%s'" % self.params.control.frame.id
        )
        self.get_logger().info("fixed string is: '%s'" % self.params.fixed_string)

        self.get_logger().info(
            "Original joints parameter is: '%s'" % str(self.params.joints)
        )
        for d in self.params.fixed_array:
            self.get_logger().info("value: '%s'" % str(d))

    def timer_callback(self):
        if self.param_listener.is_old(self.params):
            self.param_listener.refresh_dynamic_parameters()
            self.params = self.param_listener.get_params()
            self.get_logger().info(
                "New control frame parameter is: '%s'" % self.params.control.frame.id
            )
            self.get_logger().info("fixed string is: '%s'" % self.params.fixed_string)
            for d in self.params.fixed_array:
                self.get_logger().info("value: '%s'" % str(d))


def main(args=None):
    rclpy.init(args=args)
    node = MinimalParam()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
