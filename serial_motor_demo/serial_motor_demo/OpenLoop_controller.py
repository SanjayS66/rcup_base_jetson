#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time

class CmdvelToMcu(Node):
    def __init__(self):
        super().__init__('cmdvel_to_mcu')

        # --- Parameters ---
        self.declare_parameter('wheel_L', 0.305)
        self.declare_parameter('wheel_W', 0.2175)
        self.declare_parameter('max_pwm', 19)
        self.declare_parameter('scale_factor', 100.0)
        self.declare_parameter('min_pwm_threshold_normal', 16)
        self.declare_parameter('min_pwm_threshold_strafe', 28)
        self.declare_parameter('ramp_step', 12) 
        self.declare_parameter('strafe_gain', 1.8) 
        self.declare_parameter('idle_timeout', 0.05)
        self.declare_parameter('cmd_vel_in_topic', 'cmd_vel_out')
        self.declare_parameter('mcu_out_topic', 'mcu/out')
        
        # --- DYNAMIC BRAKING PARAMETERS ---
        self.declare_parameter('brake_duration_normal', 0.32)   # Duration for linear
        self.declare_parameter('brake_duration_rotation', 0.17) # Duration for turns
        
        self.declare_parameter('brake_scale_normal', 0.8)   
        self.declare_parameter('brake_scale_rotation', 0.3) 
        
        self.declare_parameter('brake_min_pwm', 15)         
        self.declare_parameter('brake_max_pwm', 30)         

        # Load Params
        self.L = float(self.get_parameter('wheel_L').get_parameter_value().double_value)
        self.W = float(self.get_parameter('wheel_W').get_parameter_value().double_value)
        self.max_pwm = int(self.get_parameter('max_pwm').get_parameter_value().integer_value)
        self.strafe_gain = float(self.get_parameter('strafe_gain').get_parameter_value().double_value)
        self.idle_timeout = float(self.get_parameter('idle_timeout').get_parameter_value().double_value)
        
        self.min_normal = int(self.get_parameter('min_pwm_threshold_normal').get_parameter_value().integer_value)
        self.min_strafe = int(self.get_parameter('min_pwm_threshold_strafe').get_parameter_value().integer_value)
        
        # Load Brake Durations
        self.brake_dur_normal = float(self.get_parameter('brake_duration_normal').get_parameter_value().double_value)
        self.brake_dur_rotation = float(self.get_parameter('brake_duration_rotation').get_parameter_value().double_value)
        
        self.brake_scale_normal = float(self.get_parameter('brake_scale_normal').get_parameter_value().double_value)
        self.brake_scale_rotation = float(self.get_parameter('brake_scale_rotation').get_parameter_value().double_value)
        self.brake_min = int(self.get_parameter('brake_min_pwm').get_parameter_value().integer_value)
        self.brake_max = int(self.get_parameter('brake_max_pwm').get_parameter_value().integer_value)

        cmd_topic = self.get_parameter('cmd_vel_in_topic').get_parameter_value().string_value
        mcu_out_topic = self.get_parameter('mcu_out_topic').get_parameter_value().string_value

        self.pub_mcu_out = self.create_publisher(String, mcu_out_topic, 10)
        self.sub_cmd = self.create_subscription(Twist, cmd_topic, self.cb_cmdvel, 20)

        self.last_cmd_time = None
        self.current_pwms = [0, 0, 0, 0] 
        self.last_moving_pwms = [0, 0, 0, 0] 

        self.is_braking = False
        self.brake_start_time = 0.0
        self.active_brake_pwm = 0            
        self.active_brake_duration = 0.0     # Dynamic duration
        self.last_motion_was_rotation = False 

        self.create_timer(0.05, self._control_loop)
        
        self.get_logger().info("Active. Dynamic Multi-Factor Braking Enabled.")
        self.send_pwm([0,0,0,0])

    def send_pwm(self, pwms):
        msg = {"pwm1": int(pwms[2]) , "pwm2": int(pwms[1]), "pwm3": int(pwms[0]), "pwm4": -int(pwms[3])}
        out = String()
        out.data = json.dumps(msg)
        self.pub_mcu_out.publish(out)

    def cb_cmdvel(self, msg: Twist):
        self.last_cmd_time = time.time()
        
        # 1. Read raw inputs using standard ROS coordinates (No inverted signs)
        raw_vx = float(msg.linear.x)
        raw_vy = float(msg.linear.y)
        wz = float(msg.angular.z)

        # Calculate strafe ratio to apply dynamic gains
        total_linear_mag = abs(raw_vx) + abs(raw_vy)
        strafe_ratio = abs(raw_vy) / total_linear_mag if total_linear_mag >= 0.05 else 0.0
        
        # Apply strafe gain to Y axis
        vx = raw_vx
        vy = raw_vy * (1.0 + (strafe_ratio * (self.strafe_gain - 1.0)))
        geom = self.L + self.W
        
        # 2. Standard Mecanum Kinematics
        # Order: [Front-Left, Front-Right, Rear-Left, Rear-Right]
        target_speeds_ms = [
            vx - vy - wz * geom,  # Index 0: FL
            vx + vy + wz * geom,  # Index 1: FR
            vx + vy - wz * geom,  # Index 2: RL
            vx - vy + wz * geom   # Index 3: RR
        ]

        # Calculate dynamic PWM thresholds based on strafing
        active_min_pwm = self.min_normal + (strafe_ratio * (self.min_strafe - self.min_normal))
        active_max_pwm = self.max_pwm + (strafe_ratio * ((self.max_pwm * self.strafe_gain) - self.max_pwm))
        NAV2_MAX_SPEED_MS = 0.09 

        target_pwms = []
        is_moving_command = False

        # 3. Convert target speeds to PWM values
        for speed_ms in target_speeds_ms:
            abs_speed = abs(speed_ms)
            if abs_speed < 0.01:
                target_pwms.append(0)
                continue
            
            is_moving_command = True
            
            # Map speed to PWM range
            speed_ratio = max(0.0, min(1.0, (max(0.02, abs_speed) - 0.02) / (NAV2_MAX_SPEED_MS - 0.02)))
            pwm_mag = active_min_pwm + (speed_ratio * (active_max_pwm - active_min_pwm))
            
            # Reapply the sign (direction)
            target_pwms.append(int(pwm_mag) if speed_ms > 0 else -int(pwm_mag))

        # Ensure the array always has exactly 4 elements
        while len(target_pwms) < 4: 
            target_pwms.append(0)

        # 4. Command the motors or trigger braking
        if is_moving_command:
            self.last_motion_was_rotation = (abs(raw_vx) < 0.02 and abs(raw_vy) < 0.02 and abs(wz) > 0.01)
            
            if self.is_braking: 
                self.is_braking = False
            
            # Pass the clean array directly without shuffling indices
            self.current_pwms = target_pwms 
            self.send_pwm(self.current_pwms)
        else:
            self.trigger_brake()

    def trigger_brake(self):
        was_moving = any(abs(p) > 0 for p in self.current_pwms)
        if was_moving and not self.is_braking:
            self.is_braking = True
            self.brake_start_time = time.time()
            self.last_moving_pwms = self.current_pwms
            
            max_prev = max([abs(p) for p in self.last_moving_pwms])
            
            if self.last_motion_was_rotation:
                factor = self.brake_scale_rotation
                self.active_brake_duration = self.brake_dur_rotation
            else:
                factor = self.brake_scale_normal
                self.active_brake_duration = self.brake_dur_normal
            
            self.active_brake_pwm = max(self.brake_min, min(self.brake_max, int(max_prev * factor)))
            self.get_logger().info(f"Brake: Force={self.active_brake_pwm}, Dur={self.active_brake_duration}")

    def _control_loop(self):
        if self.is_braking:
            if (time.time() - self.brake_start_time) < self.active_brake_duration:
                brake_pwms = [(-self.active_brake_pwm if p > 0 else self.active_brake_pwm if p < 0 else 0) for p in self.last_moving_pwms]
                self.send_pwm(brake_pwms)
            else:
                self.is_braking = False
                self.current_pwms = [0,0,0,0]
                self.send_pwm([0,0,0,0])
        elif self.last_cmd_time is not None:
             if (time.time() - self.last_cmd_time) > self.idle_timeout:
                self.trigger_brake()
                self.last_cmd_time = None

def main(args=None):
    rclpy.init(args=args)
    node = CmdvelToMcu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_pwm([0,0,0,0])
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
