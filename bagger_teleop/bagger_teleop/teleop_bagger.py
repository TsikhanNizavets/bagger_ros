"""
Multiplexer (bucket_wheel_mux.py) manages bucket wheel message priorities.

container_unloader.py stop-messages to the wheel have higher priority than teleop_bagger.py's.

Prevents ore on mechanical parts while unloading.
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from control_msgs.msg import MultiDOFCommand
from bagger_interfaces.action import UnloadContainer            

HELF_TEXT = """
===============================================
  Lunar Bucket Wheel Excavator Control
===============================================
  BASE DRIVING:
      w : Forward         s : Backward
      a : Turn Left       d : Turn Right
    space : Stop Base Movement

  ARM ELEVATION:
      r : Raise Arm       f : Lower Arm

  BUCKET WHEEL:
      e : Start/Spin Wheel  q : Stop Wheel

  UNLOAD CONTAINER:
      u : Unload          c : Cancel

  CTRL-C to quit
===============================================
"""

class ExcavetorTeleop(Node):
    def __init__(self):
        super().__init__('excavetor_teleop')

        self.pub_cmd_vel = self.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)
        self.pub_arm = self.create_publisher(Float64, '/arm_cmd', 10)
        self.pub_wheel = self.create_publisher(MultiDOFCommand, '/bucket_wheel_cmd_teleop', 10)

        self._action_client = ActionClient(self, UnloadContainer, '/unload_container')
        self._goal_handle = None

        self.linear_speed = 0.5
        self.agular_speed = 1.0

        self.arm_position = 0.0
        self.arm_step = 0.05
        self.min_arm_position = -0.75
        self.max_arm_position = 0.75

        self.wheel_spin_rate = 3.0

        self.is_moving = False

    def run_loop(self):
            settings = termios.tcgetattr(sys.stdin)
    
            print(HELF_TEXT)
    
            try:
                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.0)

                    key = self.getKey(settings)

                    #Driving
                    cmd_msg = TwistStamped()
                    cmd_msg.header.stamp = self.get_clock().now().to_msg()
                    cmd_msg.header.frame_id = "base_link"

                    if key in ['w', 's', 'a', 'd']:
                        self.is_moving = True
                        if key == 'w':
                            cmd_msg.twist.linear.x = self.linear_speed
                            print("\r\033[2K[Base] Moving Forward ", end="")
                        elif key == 's':
                            cmd_msg.twist.linear.x = -self.linear_speed
                            print("\r\033[2K[Base] Moving Backward", end="")
                        elif key == 'a':
                            cmd_msg.twist.angular.z = self.agular_speed
                            print("\r\033[2K[Base] Turning Left   ", end="")
                        elif key == 'd':
                            cmd_msg.twist.angular.z = -self.agular_speed
                            print("\r\033[2K[Base] Turning Right  ", end="")
                        self.pub_cmd_vel.publish(cmd_msg)

                    elif key == ' ':
                        self.pub_cmd_vel.publish(TwistStamped())
                        self.is_moving = False
                        print("\r\033[2K[Base] STOPPED        ", end="")

                    elif self.is_moving:
                        self.is_moving = False
                        self.pub_cmd_vel.publish(TwistStamped())
                        print("\r\033[2K[Base] Auto Stopped   ", end="")
    
                    #Arm Movement
                    elif key == 'r':
                        self.arm_position = ExcavetorTeleop.update_val(self.arm_position, self.arm_step, self.min_arm_position, self.max_arm_position)
                        msg = Float64()
                        msg.data = self.arm_position
                        self.pub_arm.publish(msg)
                        print(f"\r\033[2K[Arm] Raised -> Position: {self.arm_position:.2f} rad", end="")
                    elif key == 'f':
                        self.arm_position = ExcavetorTeleop.update_val(self.arm_position, -self.arm_step, self.min_arm_position, self.max_arm_position)
                        msg = Float64()
                        msg.data = self.arm_position
                        self.pub_arm.publish(msg)
                        print(f"\r\033[2K[Arm] Lowered -> Position: {self.arm_position:.2f} rad", end="")
    
                    #Bucket Wheel Movement
                    elif key == 'e':
                        msg = MultiDOFCommand()
                        msg.dof_names = ['bucket_wheel_joint']
                        msg.values = [self.wheel_spin_rate]
                        self.pub_wheel.publish(msg)
                        print(f"\r\033[2K[Wheel] SPINNING @ {self.wheel_spin_rate} rad/s", end="")
                    elif key == 'q':
                        msg = MultiDOFCommand()
                        msg.dof_names = ['bucket_wheel_joint']
                        msg.values = [0.0]
                        self.pub_wheel.publish(msg)
                        print(f"\r\033[2K[Wheel] STOPPED            ", end="")

                    #Unload Container                             
                    elif key == 'u':
                        self.trigger_unload()
                    elif key == 'c':
                        self.cancel_unload()
                    
                    elif key == '\x03':
                        break
    
            except Exception as e:
                print(e)
    
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def trigger_unload(self):                                               
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            print('\r\033[2K[Action] Server /unload_container not available!', end='')
            return

        goal_msg = UnloadContainer.Goal()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        print('\r\033[2K[Action] Sent unloading goal request...', end='')

    def cancel_unload(self):                                                               
        if self._goal_handle is not None:
            print('\r\033[2K[Action] Canceling unloading process...', end='')
            self._goal_handle.cancel_goal_async()

    def goal_response_callback(self, future):                                   
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            print('\r\033[2K[Action] Goal rejected by server.', end='')
            return

        print('\r\033[2K[Action] Goal accepted, unloading in progress...', end='')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):                                          
        feedback = feedback_msg.feedback
        print(f'\r\033[2K[Action Feedback] State: {feedback.current_state}', end='')

    def get_result_callback(self, future):                                              
        result = future.result().result
        print(f'\r\033[2K[Action Finished] Result: {result.message}', end='')
        self._goal_handle = None
        
    def getKey(self, settings):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    @staticmethod
    def clamp(value, min_val, max_val):
        return max(min_val, min(max_val, value))

    @staticmethod
    def update_val(current_val, step, min_val, max_val):
        return ExcavetorTeleop.clamp(current_val + step, min_val, max_val)

def main(args=None):
    rclpy.init(args=args)
    node = ExcavetorTeleop()
    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        