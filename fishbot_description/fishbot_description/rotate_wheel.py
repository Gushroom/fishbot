import rclpy
import time
from threading import Thread
from rclpy.node import Node
from sensor_msgs.msg import JointState

class RotateWheelNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"Node {name} initializing...")

        self.pub_joint_states = self.create_publisher(JointState, "joint_state", 10)

        self._init_joint_states()
        self.pub_rate = self.create_rate(30)
        self.thread_ = Thread(target=self._thread_pub)
        self.thread_.start()

    def _init_joint_states(self):
        '''
        Can be found by # ros2 interface
            std_msgs/Header header #timestamp and frame_id
            string[] name
            float64[] position
            float64[] velocity
            float64[] effort
        '''
        self.joint_speeds = [0.0, 0.0]
        self.joint_states = JointState()
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.header.frame_id = ""
        # Joint names
        self.joint_states.name = ['left_wheel_joint','right_wheel_joint']
        # Initial position
        self.joint_states.position = [0.0,0.0]
        # speeds
        self.joint_states.velocity = self.joint_speeds
        # effort
        self.joint_states.effort = []
    
    def update_speed(self,speeds):
        self.joint_speeds = speeds

    def _thread_pub(self):
        last_update_time = time.time()
        while rclpy.ok():
            time_delta = time.time() - last_update_time
            last_update_time = time.time()
            # New postion += time_delta * velocity
            self.joint_states.position[0] += time_delta * self.joint_states.velocity[0]
            self.joint_states.position[1] += time_delta * self.joint_states.velocity[1] 
            # update velocity
            self.joint_states.velocity = self.joint_speeds
            # update time
            self.joint_states.header.stamp = self.get_clock().now().to_msg()
            # publish
            self.pub_joint_states.publish(self.joint_states)
            self.pub_rate.sleep()


    
def main(args=None):
    rclpy.init(args=args)
    rotate_wheel_node = RotateWheelNode("rotate_fishbot_wheel")
    rotate_wheel_node.update_speed([15.0,-15.0])
    rclpy.spin(rotate_wheel_node)
    rclpy.shutdown()