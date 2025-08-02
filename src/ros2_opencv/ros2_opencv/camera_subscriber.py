import cv2

import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class CameraSubscriberNode(Node):
    
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridgeObject = CvBridge()

        self.topicNameFrame = 'topic_camera_image'

        self.queueSize = 20

        self.subscription = self.create_subscription(
            Image,
            self.topicNameFrame,
            self.image_callbackFunction,
            self.queueSize
        )

        self.bridgeObject = CvBridge()

    def image_callbackFunction(self, msg):

        self.get_logger().info('The image frame is received')



        frame = self.bridgeObject.imgmsg_to_cv2(msg)

        cv2.imshow('Camera Video', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    cameraSubscriberNode = CameraSubscriberNode()

    rclpy.spin(cameraSubscriberNode)

    cameraSubscriberNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()