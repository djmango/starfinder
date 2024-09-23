import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from glob import glob

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_frames', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 image per second
        self.bridge = CvBridge()
        
        # Path to the folder containing images
        self.image_folder = '/home/noopur/ros2_video_ws/src/video_transport/video_transport/sky/'
        
        # Get list of image files
        self.image_files = sorted(glob(os.path.join(self.image_folder, '*.jpg')))  # Adjust file extension if needed
        
        if not self.image_files:
            self.get_logger().error('No image files found in the specified folder')
            rclpy.shutdown()
            return
        
        self.current_image_index = 0
        self.get_logger().info(f'Found {len(self.image_files)} images')

    def timer_callback(self):
        if self.current_image_index < len(self.image_files):
            image_path = self.image_files[self.current_image_index]
            frame = cv2.imread(image_path)
            
            if frame is not None:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing image: {image_path}')
                self.current_image_index += 1
            else:
                self.get_logger().warn(f'Failed to read image: {image_path}')
        else:
            self.get_logger().info('All images have been published. Restarting from the beginning.')
            self.current_image_index = 0

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()