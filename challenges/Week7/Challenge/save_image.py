import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.get_logger().info('Waiting for image on /camera/image_raw...')

    def listener_callback(self, msg):
        self.get_logger().info('Received image! Saving...')
        # Convert image to numpy array
        # assuming msg is rgb8 or bgr8
        height = msg.height
        width = msg.width
        # We can reconstruct it from the byte array
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, -1)
        # Convert RGB to BGR if it's RGB
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imwrite('/home/alfonso/.gemini/antigravity/brain/1e625e01-e0f6-48d2-9bfe-7d775cbdc983/artifacts/camera_frame.png', img)
        self.get_logger().info('Saved image successfully!')
        # shutdown
        rclpy.shutdown()

def main():
    rclpy.init()
    saver = ImageSaver()
    try:
        rclpy.spin(saver)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
