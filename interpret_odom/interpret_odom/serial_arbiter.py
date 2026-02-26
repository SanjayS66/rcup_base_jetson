# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from threading import Thread, Lock
# import serial
# import json
# import time

# class SerialArbiter(Node):
#     def __init__(self):
#         super().__init__('serial_arbiter')
       
#         # -------- Parameters --------
#         self.declare_parameter('imu_port', '/dev/stm_imu_usb')
#         self.declare_parameter('enc_port', '/dev/stm_encoder_usb')
#         self.declare_parameter('baud', 115200)
#         self.declare_parameter('out_topic', 'mcu/in')
#         self.declare_parameter('dual_mcu', True)

#         self.imu_port = self.get_parameter('imu_port').value
#         self.enc_port = self.get_parameter('enc_port').value
#         self.baud = self.get_parameter('baud').value
#         self.out_topic = self.get_parameter('out_topic').value
#         self.dual_mcu = self.get_parameter('dual_mcu').value

#         self.get_logger().info(f"Arbiter Loaded (Dual Mode: {self.dual_mcu})")

#         # -------- State --------
#         self.imu_ser = None
#         self.enc_ser = None
        
#         # Use persistent state dictionaries instead of None flags
#         self.state_imu = {}
#         self.state_enc = {}
        
#         self.lock = Lock()
#         self.running = True

#         # -------- ROS Publishers & Subscribers --------
#         self.pub = self.create_publisher(String, self.out_topic, 20)
#         self.sub_out = self.create_subscription(String, 'mcu/out', self.cb_out, 20)

#         # -------- Threads --------
#         if self.dual_mcu:
#             Thread(target=self._imu_reader, daemon=True).start()
       
#         Thread(target=self._enc_reader, daemon=True).start()

#     def _open_serial(self, port):
#         try:
#             ser = serial.Serial(port, self.baud, timeout=0.2)
#             time.sleep(0.5)
#             ser.reset_input_buffer()
#             self.get_logger().info(f"Successfully opened {port}")
#             return ser
#         except Exception as e:
#             self.get_logger().error(f"Failed to open {port}: {e}")
#             return None

#     def _send_stop(self, ser_obj):
#         if ser_obj and ser_obj.is_open:
#             try:
#                 stop_msg = '{"pwm1": 0, "pwm2": 0, "pwm3": 0, "pwm4": 0}\n'
#                 ser_obj.write(stop_msg.encode())
#                 ser_obj.flush()
#                 time.sleep(0.1)
#             except Exception as e:
#                 pass

#     def cb_out(self, msg: String):
#         payload = msg.data if msg.data.endswith("\n") else msg.data + "\n"
#         with self.lock:
#             target_ser = self.enc_ser
            
#         if target_ser and target_ser.is_open:
#             try:
#                 target_ser.write(payload.encode())
#                 target_ser.flush()
#             except serial.SerialException as e:
#                 self.get_logger().error(f"Write failed, dropping connection: {e}")
#                 with self.lock:
#                     self.enc_ser.close()
#                     self.enc_ser = None

#     def _imu_reader(self):
#         last_rx_time = time.time()
#         while self.running:
#             with self.lock:
#                 current_ser = self.imu_ser

#             if not current_ser or not current_ser.is_open:
#                 time.sleep(1.0)
#                 new_ser = self._open_serial(self.imu_port)
#                 with self.lock:
#                     self.imu_ser = new_ser
#                 if new_ser:
#                     last_rx_time = time.time()
#                 continue

#             try:
#                 # Only flush if the buffer is actually backed up (prevents blocking)
#                 if current_ser.in_waiting > 256:
#                     current_ser.reset_input_buffer()
#                     current_ser.readline() 
                
#                 line = current_ser.readline().decode(errors='ignore').strip()
                
#                 if not line:
#                     if time.time() - last_rx_time > 2.0:
#                         with self.lock:
#                             self.imu_ser.close()
#                             self.imu_ser = None
#                     continue

#                 last_rx_time = time.time()
#                 if "{" not in line: continue
                
#                 data = json.loads(line[line.find("{"):])
                
#                 # Update persistent state
#                 with self.lock:
#                     self.state_imu = data
                
#                 # Publish immediately, don't wait for encoder
#                 self._try_publish()

#             except (serial.SerialException, OSError, json.JSONDecodeError):
#                 pass

#     def _enc_reader(self):
#         last_rx_time = time.time()
#         while self.running:
#             with self.lock:
#                 current_ser = self.enc_ser

#             if not current_ser or not current_ser.is_open:
#                 time.sleep(1.0)
#                 new_ser = self._open_serial(self.enc_port)
#                 with self.lock:
#                     self.enc_ser = new_ser
#                 if new_ser:
#                     self._send_stop(new_ser)
#                     last_rx_time = time.time()
#                 continue

#             try:
#                 # Only flush if the buffer is actually backed up
#                 if current_ser.in_waiting > 256:
#                     current_ser.reset_input_buffer()
#                     current_ser.readline()
                
#                 line = current_ser.readline().decode(errors='ignore').strip()
                
#                 if not line:
#                     if time.time() - last_rx_time > 2.0:
#                         with self.lock:
#                             self.enc_ser.close()
#                             self.enc_ser = None
#                     continue

#                 last_rx_time = time.time()
#                 if "{" not in line: continue
                
#                 data = json.loads(line[line.find("{"):])
                
#                 # Update persistent state
#                 with self.lock:
#                     self.state_enc = data
                
#                 # Publish immediately, don't wait for IMU
#                 self._try_publish()

#             except (serial.SerialException, OSError, json.JSONDecodeError):
#                 pass

#     def _try_publish(self):
#         with self.lock:
#             # Require at least one initial message from required sensors to start
#             if not self.state_enc or (self.dual_mcu and not self.state_imu):
#                 return

#             # Merge the latest known states of both sensors
#             merged = {}
#             if self.dual_mcu:
#                 merged.update(self.state_imu)
#             merged.update(self.state_enc)
            
#             # Note: We NO LONGER clear the states here. We keep caching them.

#         msg = String()
#         msg.data = json.dumps(merged)
#         self.pub.publish(msg)

#     def destroy_node(self):
#         self.running = False
#         with self.lock:
#             self._send_stop(self.enc_ser)
#             for s in [self.imu_ser, self.enc_ser]:
#                 if s and s.is_open: 
#                     s.close()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialArbiter()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok(): rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread, Lock
import serial
import json
import time

class SerialArbiter(Node):
    def __init__(self):
        super().__init__('serial_arbiter')
       
        # -------- Parameters --------
        self.declare_parameter('imu_port', '/dev/stm_imu_usb')
        self.declare_parameter('enc_port', '/dev/stm_encoder_usb')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('out_topic', 'mcu/in')
        self.declare_parameter('dual_mcu', True)

        self.imu_port = self.get_parameter('imu_port').value
        self.enc_port = self.get_parameter('enc_port').value
        self.baud = self.get_parameter('baud').value
        self.out_topic = self.get_parameter('out_topic').value
        self.dual_mcu = self.get_parameter('dual_mcu').value

        self.get_logger().info(f"Arbiter Loaded (Dual Mode: {self.dual_mcu})")

        # -------- State --------
        self.imu_ser = None
        self.enc_ser = None
        self.latest_imu = None
        self.latest_enc = None
        self.lock = Lock()
        self.running = True

        # -------- ROS Publishers & Subscribers --------
# -------- ROS Publishers & Subscribers --------
        # Replace the single pub with two separate publishers
        self.pub_imu = self.create_publisher(String, 'mcu/imu', 20)
        self.pub_enc = self.create_publisher(String, 'mcu/enc', 20)
        self.sub_out = self.create_subscription(String, 'mcu/out', self.cb_out, 20)
        # -------- Threads --------
        if self.dual_mcu:
            Thread(target=self._imu_reader, daemon=True).start()
       
        Thread(target=self._enc_reader, daemon=True).start()

    def _open_serial(self, port):
        try:
            # 0.2s timeout prevents readline() from blocking forever if cable drops
            ser = serial.Serial(port, self.baud, timeout=0.2)
            time.sleep(0.5)
            ser.reset_input_buffer()
            self.get_logger().info(f"Successfully opened {port}")
            return ser
        except Exception as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            return None

    def _send_stop(self, ser_obj):
        if ser_obj and ser_obj.is_open:
            try:
                stop_msg = '{"pwm1": 0, "pwm2": 0, "pwm3": 0, "pwm4": 0}\n'
                ser_obj.write(stop_msg.encode())
                ser_obj.flush()
                time.sleep(0.1)
                self.get_logger().info("Zero PWM command sent.")
            except Exception as e:
                self.get_logger().warn(f"Failed to send zero PWM: {e}")

    def cb_out(self, msg: String):
        payload = msg.data if msg.data.endswith("\n") else msg.data + "\n"
        with self.lock:
            target_ser = self.enc_ser
            
        if target_ser and target_ser.is_open:
            try:
                target_ser.write(payload.encode())
                target_ser.flush()
            except serial.SerialException as e:
                self.get_logger().error(f"Write failed, dropping connection: {e}")
                # Force close so the read thread triggers a reconnect
                with self.lock:
                    self.enc_ser.close()
                    self.enc_ser = None

    def _imu_reader(self):
        last_rx_time = time.time()
        while self.running:
            with self.lock:
                current_ser = self.imu_ser

            if not current_ser or not current_ser.is_open:
                time.sleep(1.0)
                new_ser = self._open_serial(self.imu_port)
                with self.lock:
                    self.imu_ser = new_ser
                if new_ser:
                    last_rx_time = time.time()
                continue

            try:
                # 1. Wipe out the backlog of old queued data
                current_ser.reset_input_buffer()
                
                # 2. Read and discard the likely truncated partial line
                current_ser.readline() 
                
                # 3. Read the first complete, fully fresh line
                line = current_ser.readline().decode(errors='ignore').strip()
                
                # Check for dead connection
                if not line:
                    if time.time() - last_rx_time > 2.0:
                        self.get_logger().warn("IMU timeout. Forcing reconnect...")
                        with self.lock:
                            self.imu_ser.close()
                            self.imu_ser = None
                    continue

                last_rx_time = time.time()
                if "{" not in line: continue
                data = json.loads(line[line.find("{"):])
                
                msg = String()
                msg.data = line[line.find("{"):]
                self.pub_imu.publish(msg)

            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"IMU Serial Error: {e}")
                with self.lock:
                    if self.imu_ser:
                        self.imu_ser.close()
                        self.imu_ser = None
            except json.JSONDecodeError:
                pass # Ignore malformed partial strings

    def _enc_reader(self):
        last_rx_time = time.time()
        
        while self.running:
            with self.lock:
                current_ser = self.enc_ser

            if not current_ser or not current_ser.is_open:
                time.sleep(1.0)
                new_ser = self._open_serial(self.enc_port)
                with self.lock:
                    self.enc_ser = new_ser
                if new_ser:
                    self._send_stop(new_ser)
                    last_rx_time = time.time()
                continue

            try:
                # 1. Wipe out the backlog of old queued data
                current_ser.reset_input_buffer()
                
                # 2. Read and discard the likely truncated partial line
                current_ser.readline() 
                
                # 3. Read the first complete, fully fresh line
                line = current_ser.readline().decode(errors='ignore').strip()
                
                # Check for dead connection
                if not line:
                    if time.time() - last_rx_time > 2.0:
                        self.get_logger().warn("Encoder timeout. Forcing reconnect...")
                        with self.lock:
                            self.enc_ser.close()
                            self.enc_ser = None
                    continue

                last_rx_time = time.time()
                if "{" not in line: continue
                data = json.loads(line[line.find("{"):])
                
                msg = String()
                msg.data = line[line.find("{"):]
                self.pub_enc.publish(msg)

            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"Encoder Serial Error: {e}")
                with self.lock:
                    if self.enc_ser:
                        self.enc_ser.close()
                        self.enc_ser = None
            except json.JSONDecodeError:
                pass # Ignore malformed partial strings

    def destroy_node(self):
        self.get_logger().info("Shutting down... sending EMERGENCY STOP to MCU.")
        self.running = False
       
        with self.lock:
            self._send_stop(self.enc_ser)
            for s in [self.imu_ser, self.enc_ser]:
                if s and s.is_open: 
                    s.close()
       
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()