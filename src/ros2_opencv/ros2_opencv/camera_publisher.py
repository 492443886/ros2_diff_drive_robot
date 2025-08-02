import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class CameraPublisherNode(Node):

    def __init__(self):
        super().__init__('camera_publisher_node')
        self.cameraDevice = 0  # Default camera device
        self.camera = cv2.VideoCapture(self.cameraDevice)

        # self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 820)
        # self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
        # self.camera.set(cv2.CAP_PROP_FPS, 30)  # Or 60, if supported

        self.bridgeObject = CvBridge()

        self.topicNameFrame = 'topic_camera_image'

        self.queueSize = 20

        self.publisher = self.create_publisher(Image, self.topicNameFrame, self.queueSize)

        self.periodCommunication = 1/30.0  # 30 Hz

        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)

        self.i = 0

    def timer_callbackFunction(self):

        success, frame = self.camera.read()
        frame = cv2.resize(frame, (820, 640), interpolation=cv2.INTER_CUBIC)

        if success:

            ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(frame)
            self.publisher.publish(ROS2ImageMessage)
            self.get_logger().info('Publishing image number %d' % self.i )

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    cameraPublisherNode = CameraPublisherNode()

    rclpy.spin(cameraPublisherNode)

    cameraPublisherNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()