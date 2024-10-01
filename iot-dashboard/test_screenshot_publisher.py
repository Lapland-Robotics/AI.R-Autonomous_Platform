from PIL import Image as PILImage
import io
import base64
import json
import os
from azure.iot.device import IoTHubDeviceClient
from azure.iot.device.exceptions import ConnectionFailedError
import time

# Azure IoT Hub configuration with device "snower"
with open("connect_string.txt", "r") as f:
    CONNECTION_STRING = f.read().strip('"')


#let's try taking screenshots, formatting them and sending them to the cloud
# we will not use ROS in this case
import pyscreenshot as ImageGrab

def resize_image(image, max_size=(800, 600)):
    """Resize image to fit within max_size without exceeding it"""
    image.thumbnail(max_size, PILImage.LANCZOS)
    return image

    # Check payload size

# let's try to connect to the Azure IoT Hub
# Initialize IoT Hub client
try:
    client = IoTHubDeviceClient.create_from_connection_string(CONNECTION_STRING)
    print("Connected to Azure IoT Hub.")
except ConnectionFailedError:
    print("Failed to connect to Azure IoT Hub.")
    client = None

# take a screenshot
img = ImageGrab.grab()

# Resize image if needed
resized_image = resize_image(img)

# Convert the image to a base64 string
buffered = io.BytesIO()
resized_image.save(buffered, format="JPEG")
image_base64 = base64.b64encode(buffered.getvalue()).decode('utf-8')

# Create JSON payload
payload = json.dumps({
    "image_data": image_base64
})

# Check payload size
print("Payload size: ", len(payload))

# now that we have the client initialized, let's try to send the payload
if client:
    for i in range(5):
        try:
            client.send_message(payload)
            print("Message sent to Azure IoT Hub.")
        except ConnectionFailedError:
            print("Failed to send message to Azure IoT Hub.")
        time.sleep(2)

    client.shutdown()
    print("Disconnected from Azure IoT Hub.")
