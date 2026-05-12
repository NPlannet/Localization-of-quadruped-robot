import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class NaoJointMover(Node):
    def __init__(self):
        super().__init__('nao_joint_mover')
        
         
# New (ROS 2 friendly):
        topic_name = '/model/Nao/joint/LShoulderPitch/cmd_pos'
        
        self.publisher_ = self.create_publisher(Float64, topic_name, 10)
        
        # Timer set to 20Hz (0.05 seconds) for smooth motion
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.counter = 0.0
        self.get_logger().info(f'Continuous mover started. Publishing to {topic_name}')

    def timer_callback(self):
        msg = Float64()
        
        # Use a sine wave to oscillate between -1.0 and 1.0 radians
        # math.sin returns a value between -1 and 1
        msg.data = math.sin(self.counter)
        
        self.publisher_.publish(msg)
        
        # Log the current angle every 20 iterations to avoid spamming the console
        if int(self.counter * 10) % 10 == 0:
            self.get_logger().info(f'Joint Angle: {msg.data:.2f} rad')
            
        self.counter += 0.05 # Increment to move the "wave" forward

def main():
    rclpy.init()
    node = NaoJointMover()
    
    try:
        # This keeps the node running and the timer active
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()