#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToCmdVelJoy(Node):
    """
    Converts /joy -> /cmd_vel_joy (Twist)

    Behavior:
      - If brake_button (default index 7) in msg.buttons is 1, publish zero Twist continuously.
      - Otherwise publish only when any mapped axis is outside deadzone.
    Params:
      axis_vx, axis_vy, axis_wz : indices for mapping
      scale_vx, scale_vy, scale_wz : scaling factors
      deadzone : ignore small stick noise
      publish_rate : Hz
      brake_button : index in buttons[] that acts as brake (default 7)
    """
    def __init__(self):
        super().__init__('joy_to_cmdvel_joy_safe_brake_button')

        # params
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_joy')
        self.declare_parameter('axis_vx', 1)
        self.declare_parameter('axis_vy', 0)
        self.declare_parameter('axis_wz', 2)
        self.declare_parameter('scale_vx', 1.0)
        self.declare_parameter('scale_vy', 1.0)
        self.declare_parameter('scale_wz', 1.0)
        self.declare_parameter('deadzone', 0.2)
        self.declare_parameter('publish_rate', 30.0)

        # brake uses a button index in the buttons array
        self.declare_parameter('brake_button', 7)  # 8th element => index 7

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.axis_vx = int(self.get_parameter('axis_vx').get_parameter_value().integer_value)
        self.axis_vy = int(self.get_parameter('axis_vy').get_parameter_value().integer_value)
        self.axis_wz = int(self.get_parameter('axis_wz').get_parameter_value().integer_value)
        self.scale_vx = float(self.get_parameter('scale_vx').get_parameter_value().double_value)
        self.scale_vy = float(self.get_parameter('scale_vy').get_parameter_value().double_value)
        self.scale_wz = float(self.get_parameter('scale_wz').get_parameter_value().double_value)
        self.deadzone = float(self.get_parameter('deadzone').get_parameter_value().double_value)
        self.publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self.brake_button = int(self.get_parameter('brake_button').get_parameter_value().integer_value)

        # pub/sub
        self.pub_twist = self.create_publisher(Twist, cmd_topic, 10)
        self.sub_joy = self.create_subscription(Joy, joy_topic, self.cb_joy, 50)

        # internal state
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._active = False        # true when there is meaningful stick input
        self._brake_active = False  # true when brake button pressed

        self.create_timer(1.0 / self.publish_rate, self._publish_timer)

        self.get_logger().info(f"joy_to_cmdvel_joy_safe_brake_button: '{joy_topic}' -> '{cmd_topic}' ready (brake_button={self.brake_button})")

    def _apply_deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    def _check_brake_button(self, buttons):
        # safe indexing into buttons array
        if len(buttons) <= self.brake_button:
            return False
        try:
            return bool(buttons[self.brake_button])
        except Exception:
            return False

    def cb_joy(self, msg: Joy):
        axes = msg.axes
        buttons = msg.buttons

        # safe indexing for axes
        vx_raw = axes[self.axis_vx] if len(axes) > self.axis_vx else 0.0
        vy_raw = axes[self.axis_vy] if len(axes) > self.axis_vy else 0.0
        wz_raw = axes[self.axis_wz] if len(axes) > self.axis_wz else 0.0

        # apply deadzone and scale
        vx = self._apply_deadzone(vx_raw) * self.scale_vx
        vy = self._apply_deadzone(vy_raw) * self.scale_vy
        wz = self._apply_deadzone(wz_raw) * self.scale_wz

        # determine activity: at least one axis outside deadzone
        active_axes = (abs(vx) > 0.0) or (abs(vy) > 0.0) or (abs(wz) > 0.0)
        self._active = bool(active_axes)

        # brake detection from buttons array
        self._brake_active = self._check_brake_button(buttons)

        # store values
        self._vx = float(vx)
        self._vy = float(vy)
        self._wz = float(wz)

        # debug logs
        if self._brake_active:
            self.get_logger().debug("Brake button pressed: publishing zero Twist")
        elif self._active:
            self.get_logger().debug(f"JOY active vx={self._vx:.3f} vy={self._vy:.3f} wz={self._wz:.3f}")
        else:
            self.get_logger().debug("JOY inactive: inside deadzone (not publishing)")

    def _publish_timer(self):
        # if brake button is pressed, publish zero Twist continuously
        if self._brake_active:
            t = Twist()
            t.linear.x = 0.0
            t.linear.y = 0.0
            t.angular.z = 0.0
            self.pub_twist.publish(t)
            return

        # if joystick inactive, do not publish anything (allow mux to time out)
        if not self._active:
            return

        # otherwise publish the current command
        t = Twist()
        t.linear.x = self._vx
        t.linear.y = self._vy
        t.angular.z = self._wz
        self.pub_twist.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVelJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
