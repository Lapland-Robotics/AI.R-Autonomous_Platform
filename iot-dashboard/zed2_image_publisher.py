import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from azure.iot.device import IoTHubDeviceClient
from azure.iot.device.exceptions import ConnectionFailedError
from PIL import Image as PILImage
import io
import base64
import json
import os
from dotenv import load_dotenv

# Load Enviroment Variables from .env file
load_dotenv()

# Azure IoT Hub configuration with device "snower"
CONNECTION_STRING = os.getenv("AZURE_IOT_HUB_CONNECTION_STRING")

class ZED2ImagePublisher(Node):
    def __init__(self):
        super().__init__('zed2_image_publisher')
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Create a timer to publish every 5 seconds
        self.timer = self.create_timer(5.0, self.publish_to_azure)
        
        # Initialize variables
        self.last_image = None
        self.client = None

        # Initialize IoT Hub client
        try:
            self.client = IoTHubDeviceClient.create_from_connection_string(CONNECTION_STRING)
            self.get_logger().info("Connected to Azure IoT Hub.")
        except ConnectionFailedError:
            self.get_logger().error("Failed to connect to Azure IoT Hub.")
            self.client = None

    def listener_callback(self, msg):
        # Convert ROS2 Image message to PIL Image
        self.last_image = self.ros2_to_pil_image(msg)

    def publish_to_azure(self):
        if self.last_image and self.client:
            try:
                # Resize image if needed
                resized_image = self.resize_image(self.last_image)

                # Convert the image to a base64 string
                buffered = io.BytesIO()
                resized_image.save(buffered, format="JPEG")
                image_base64 = base64.b64encode(buffered.getvalue()).decode('utf-8')
                
                # Create JSON payload
                payload = json.dumps({
                    "image_data": image_base64
                })
                
                # Check payload size
                if len(payload.encode('utf-8')) > 256 * 1024:
                    self.get_logger().error("Payload size exceeds 256 KB limit.")
                    return
                
                # Send the data to Azure IoT Hub
                self.client.send_message(payload)
                self.get_logger().info("Image sent to Azure IoT Hub.")
            except Exception as e:
                self.get_logger().error(f"Failed to send image to Azure IoT Hub: {e}")
    
    def resize_image(self, image, max_size=(800, 600)):
        """Resize image to fit within max_size without exceeding it"""
        image.thumbnail(max_size, PILImage.ANTIALIAS)
        return image

    def ros2_to_pil_image(self, ros2_image):
        # Convert ROS2 Image message to PIL Image

        image_data = ros2_image.data.tobytes()

        pil_image = PILImage.frombytes('RGB', (ros2_image.width, ros2_image.height), image_data)
        return pil_image

def main(args=None):
    rclpy.init(args=args)
    zed2_image_publisher = ZED2ImagePublisher()
    rclpy.spin(zed2_image_publisher)
    zed2_image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
