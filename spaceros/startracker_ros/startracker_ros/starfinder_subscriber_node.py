import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2         
import matplotlib.pyplot as plt

class StarfinderSubscriber(Node):
    def __init__(self):
        super().__init__('starfinder_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'starfinder_output',
            self.listener_callback,
            10)
        self.cv_bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Received an image')
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            # Load and display the image
            # image = cv2.imread('path_to_image.jpg')
            # cv2.imshow("Starfinder Output", cv_image)

            # Displaying the image in a graphplot
            plt.imshow(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            plt.axis('off')
            plt.show()

            cv2.waitKey(1)  
        except Exception as e:
            self.get_logger().error('Error processing image: {}'.format(str(e)))

def main(args=None):
    rclpy.init(args=args)
    starfinder_subscriber = StarfinderSubscriber()
    rclpy.spin(starfinder_subscriber)
    starfinder_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()