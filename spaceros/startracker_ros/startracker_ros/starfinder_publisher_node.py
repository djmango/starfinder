import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import cv2
import numpy as np

class StarfinderNode(Node):
    def __init__(self):
        super().__init__('starfinder_node')
        self.publisher_ = self.create_publisher(Image, 'starfinder_output', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.cv_bridge = CvBridge()

    def timer_callback(self):
        # Runs the Rust application and outputs the image
        result = subprocess.run(["cargo", "run", "--release"], cwd="/home/spaceros-user/starfinder", capture_output=True) # cwd is path to main repo
        image_path = "/home/spaceros-user/starfinder/renders/star_map.png"  # Update the output image/folder path 
        
        # Read the image using OpenCV
        cv_image = cv2.imread(image_path)
        
        if cv_image is not None:
            # Convert the image to a ROS message
            ros_image = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            # Publish the image
            self.publisher_.publish(ros_image)
            self.get_logger().info('Published an image')
        else:
            self.get_logger().error('Failed to read the image')

def main(args=None):
    rclpy.init(args=args)
    starfinder_node = StarfinderNode()
    rclpy.spin(starfinder_node)
    starfinder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
