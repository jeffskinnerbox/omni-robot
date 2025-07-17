#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from turtlesim.srv import TeleportAbsolute, TeleportRelative
from std_srvs.srv import Empty
import math
import time


class TurtleGamepadController(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        # Parameters
        self.declare_parameter("turtle_name", "turtle1")
        self.declare_parameter("linear_axis", 1)
        self.declare_parameter("angular_axis", 0)
        self.declare_parameter("turbo_button", 5)
        self.declare_parameter("slow_button", 4)
        self.declare_parameter("emergency_stop_button", 6)
        self.declare_parameter("reset_button", 7)
        self.declare_parameter("max_linear_speed", 2.0)
        self.declare_parameter("max_angular_speed", 2.0)
        self.declare_parameter("speed_increment", 0.1)
        self.declare_parameter("deadzone_threshold", 0.1)

        # Get parameters
        self.turtle_name = self.get_parameter("turtle_name").value
        self.linear_axis = self.get_parameter("linear_axis").value
        self.angular_axis = self.get_parameter("angular_axis").value
        self.turbo_button = self.get_parameter("turbo_button").value
        self.slow_button = self.get_parameter("slow_button").value
        self.emergency_stop_button = self.get_parameter("emergency_stop_button").value
        self.reset_button = self.get_parameter("reset_button").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.speed_increment = self.get_parameter("speed_increment").value
        self.deadzone_threshold = self.get_parameter("deadzone_threshold").value

        # State variables
        self.emergency_stopped = False
        self.current_linear_scale = 1.0
        self.current_angular_scale = 1.0
        self.last_button_state = {}
        self.last_emergency_press = 0

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, f"/{self.turtle_name}/cmd_vel", 10
        )

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # Service clients
        self.teleport_abs_client = self.create_client(
            TeleportAbsolute, f"/{self.turtle_name}/teleport_absolute"
        )

        self.clear_client = self.create_client(Empty, "/clear")

        self.get_logger().info(
            f"Turtle Gamepad Controller initialized for {self.turtle_name}"
        )
        self.get_logger().info("Controls:")
        self.get_logger().info("  Left stick: Move turtle")
        self.get_logger().info("  RB (Right Bumper): Turbo mode")
        self.get_logger().info("  LB (Left Bumper): Slow mode")
        self.get_logger().info("  BACK button: Emergency stop")
        self.get_logger().info("  START button: Reset turtle position")

    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone_threshold:
            return 0.0
        return value

    def joy_callback(self, msg):
        """Handle joystick input"""
        try:
            # Handle emergency stop
            if len(msg.buttons) > self.emergency_stop_button:
                if msg.buttons[self.emergency_stop_button] == 1:
                    current_time = time.time()
                    if current_time - self.last_emergency_press > 0.5:  # Debounce
                        self.emergency_stopped = not self.emergency_stopped
                        self.last_emergency_press = current_time
                        if self.emergency_stopped:
                            self.get_logger().warn("EMERGENCY STOP ACTIVATED!")
                            self.stop_turtle()
                        else:
                            self.get_logger().info("Emergency stop deactivated")

            # Handle reset
            if len(msg.buttons) > self.reset_button:
                if (
                    msg.buttons[self.reset_button] == 1
                    and self.last_button_state.get(self.reset_button, 0) == 0
                ):
                    self.reset_turtle()

            # Handle speed controls
            if len(msg.buttons) > self.turbo_button:
                if (
                    msg.buttons[self.turbo_button] == 1
                    and self.last_button_state.get(self.turbo_button, 0) == 0
                ):
                    self.current_linear_scale = min(
                        2.0, self.current_linear_scale + self.speed_increment
                    )
                    self.current_angular_scale = min(
                        2.0, self.current_angular_scale + self.speed_increment
                    )
                    self.get_logger().info(
                        f"Speed increased: {self.current_linear_scale:.1f}x"
                    )

            if len(msg.buttons) > self.slow_button:
                if (
                    msg.buttons[self.slow_button] == 1
                    and self.last_button_state.get(self.slow_button, 0) == 0
                ):
                    self.current_linear_scale = max(
                        0.1, self.current_linear_scale - self.speed_increment
                    )
                    self.current_angular_scale = max(
                        0.1, self.current_angular_scale - self.speed_increment
                    )
                    self.get_logger().info(
                        f"Speed decreased: {self.current_linear_scale:.1f}x"
                    )

            # Store button states for edge detection
            self.last_button_state = {i: button for i, button in enumerate(msg.buttons)}

            # Don't move if emergency stopped
            if self.emergency_stopped:
                return

            # Process movement
            if len(msg.axes) > max(self.linear_axis, self.angular_axis):
                linear_input = self.apply_deadzone(msg.axes[self.linear_axis])
                angular_input = self.apply_deadzone(msg.axes[self.angular_axis])

                # Create and publish twist message
                twist = Twist()
                twist.linear.x = (
                    linear_input * self.max_linear_speed * self.current_linear_scale
                )
                twist.angular.z = (
                    -angular_input * self.max_angular_speed * self.current_angular_scale
                )

                self.cmd_vel_pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Error in joy_callback: {str(e)}")

    def stop_turtle(self):
        """Stop the turtle immediately"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def reset_turtle(self):
        """Reset turtle to center position"""
        self.get_logger().info("Resetting turtle position...")

        # Clear the screen
        if self.clear_client.wait_for_service(timeout_sec=1.0):
            clear_request = Empty.Request()
            self.clear_client.call_async(clear_request)

        # Teleport to center
        if self.teleport_abs_client.wait_for_service(timeout_sec=1.0):
            teleport_request = TeleportAbsolute.Request()
            teleport_request.x = 5.544444561
            teleport_request.y = 5.544444561
            teleport_request.theta = 0.0
            self.teleport_abs_client.call_async(teleport_request)
            self.get_logger().info("Turtle reset to center position")
        else:
            self.get_logger().warn("Teleport service not available")


def main(args=None):
    rclpy.init(args=args)

    controller = TurtleGamepadController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
