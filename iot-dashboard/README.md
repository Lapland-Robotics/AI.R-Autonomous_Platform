**Vue Centralized Server** is a website created using the Vue framework, designed to control and monitor Jetson Orin Nano robot known as Snower. The site communicates with several Microsoft Azure services, including Azure Event Hub, Azure Stream Analytics Jobs, and Azure IoT Hub.

## Technologies & hardware used

- **[Vue.js](https://vuejs.org/)**: Frontend framework.
- **[WebSocket](https://github.com/theturtle32/WebSocket-Node)**: Two-way communication between Azure and the website.
- **[AI.R-ATV, Mini-ATV repository](https://github.com/Lapland-Robotics/AI.R-ATV)**: Documentation for the robot, that uses the same Jetson Orin Nano as a 'brain'.
- **[Azure Event Hub](https://docs.microsoft.com/azure/event-hubs/)**: Collecting events and sending them.
- **[Azure Stream Analytics](https://docs.microsoft.com/azure/stream-analytics/)**: Data analysis.
- **[Azure IoT Hub](https://docs.microsoft.com/azure/iot-hub/)**: Device communication.
- **[ROS 2, Robot Operating System](https://www.ros.org/)**: Robot communication & control.
- **[ZED 2 Camera](https://www.stereolabs.com/docs/ros2)**: ZED 2 Stereo Camera for depth data, sensor data, pose tracking etc.
- **[ZED SDK for JetPack 6.0 GA (L4T 36.3)](https://www.stereolabs.com/en-fi/developers/release)**: Connecting a camera to ROS 2 enviroment with a [zed_ros2_wrapper](https://github.com/stereolabs/zed-ros2-wrapper).


## Features

- **Real-time monitoring**: ✅/❌ Monitor Snower's ZED 2-camera's activity in real time.
- **Data Visualization**: ✅/❌ Present collected data as a string on the website.
- **Command control**: ❌ Send commands to Snower and control its operation directly from the user interface.
- **User Authentication**: ❌ The user must log in with specific usernames and passwords in order to send commands to the robot.
- **Security**: ✅ All Azure connection strings are encrypted for safety reasons inside .env-files.


✅ = Implemented

❌ = Not Implemented 


## Author's / Dev's Note

Before you start reviewing and using the project, I want you to note the following points about this project.

The project is a **work in progress**, so there may be *bugs* or other imperfections.

The purpose of the project was to create a website using the Vue framework that communicates with Azure Event Hub (sofie-event-hub). This Azure Event Hub in turn communicates with Azure Iot Hub (sofie-iot-hub) using Azure Stream Analytics Jobs (iot-data-analytics).

A ROS node named zed2_image_publisher and a ZED node are executed at the end of the robot. This connection sends data in base64 format to Azure Iot Hub.

However, the connection between the robot and the site is not perfect. The data remains in Azure, and for the time being it does not move on from there. There is currently no information as to why the data is not moving forward. Problem solving and help is greatly appreciated.

![Azure Full Architecture (3)](https://github.com/user-attachments/assets/8febf0e5-d8ba-4a54-b701-002ec469c6ca)

## Project setup
```
npm install
```

### Compiles and hot-reloads for development & runs server.js-node at the same time
```
npm run dev
```

### Compiles and minifies for production
```
npm run build
```

### Lints and fixes files
```
npm run lint
```


# How to make it work

## 1. Azure Side Connection

In Azure, navigate to Azure Stream Analytics Jobs (iot-data-analytics) and open Job topology from the navigation bar and then the Query section.

![eventhub](https://github.com/user-attachments/assets/def4b043-09b6-4191-829b-1b44f16324db)

Press "Start Job" in the Query section, then the data should start moving if the necessary scripts have been started in the robot, and the same on the site side. Check that you've done all the steps in the [Robot Side Connection](#2-robot-side-connection) and [Website Side Connection](#3-website-side-connection).

## 2. Robot Side Connection

One ROS2 Node is needed between the robot and Azure Iot Hub, which uses Azure Iot Hub to establish a connection. In theory, this can be done in the same way for every robot, topics and sensor data may only need editing on the code side in which format they are sent and which libraries are needed.

When using Jetson Orin Nano (Snower), you must first start the ZED 2 camera with the Node command:

```
source ~/.bashrc
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

Start another Terminal and go to the file location and source bash:

```
cd ros2_ws
source ~/ros2_ws/install/setup.bash
```


Then you run a ROS 2 Node called **zed2_image_publisher.py**:

```
ros2 run zed2_image_publisher zed2_image_publisher
```

Please note, that if you're using this file in a new robot, you need to install the **azure-iot-device** library for this code to work! You can install it with a command:

```
pip install azure-iot-device
```

## 3. Website Side Connection

Navigate to the following location in Terminal:

**C:\Users\adm.frostbit\Documents\vue-cen-ros**

In the location, run the command:

```
npm run dev
```
This command will concurrently run two commands. First command is "*npm run serve*" that starts the Vue development server and second command is "*npm run script*" that will run the server.js-node. The server.js-file creates a WebSocket server, creates EventHub clients (consumer, producer) and processes messages from Event Hub and messages sent by the user via WebSocket. 

Open up a browser and go to **localhost:8080**. Here you can see the website. Navigate to the Robot List, and select a robot (Snower) and press "View Data." You should see the following view:

![RobotDetail](https://github.com/user-attachments/assets/2cfa09e8-125f-4bd0-ad2e-2d25f1f607fc)

**Important note!**

You can't send commands to the robot (yet) with the text field. This needs more configuration in the robot's side & possibly requires that e.g. the Azure Functions service be implemented in Azure, which is configured to listen to the Event Hub and forward the data (commands) to the Azure Iot Hub.


## Adding Robots
### Azure Devices
In order to add a robot to the project, you must first define the robot on the Azure end. Go to **Azure Iot Hub (name: sofie-iot-hub)** and select "**Device Management**" from the left menu and then "**Devices**".

A view opens where you can add a new robot by pressing the "**Add Device**" button.

Note that the following robots are already defined in Azure Iot Hub:

- Rosmaster X3 Plus (rosmaster-x3-plus)
- Rosmaster X3 (rosmaster-x3)
- Big ATV (big-ATV)
- Snower (snower)
- Mini-ATV (mini-ATV)

### Webpage (Vue Centralized Server)

To add a new robot to the list, you should navigate to the following location in the project:

**C:\Users\adm.frostbit\Documents\vue-cen-ros\src\views\RobotList.vue**

Scroll down the file a bit and you will see the **<script>** tag, where **RobotList** is defined as the name of the component. A data method is defined inside the tag, which returns the robot array called"**robots**".

![robotlisttrue](https://github.com/user-attachments/assets/bb7ee06a-124d-4c88-9771-b6be15b8a29b)

Add a new robot to this list by copying one line and replace the robot's information with it:

```
{ name: '<Robot Name>', rosVersion: '<Robot's ROS Version>', status: <false / true>}
```

The robot's boolean value status is defined manually here, and in practice it is only information for the user/developer, whether the robot in question has been defined in such a way that it is possible to use it directly in the Vue Centralized Server, i.e. whether it is connected from the robot to Azure and from Azure to the site.

Please note that status **won't automatically update** once you have connected the robot to the Azure Iot Hub! You need to do it manually.

## Project Architecture

![Azure Full Architecture (1)](https://github.com/user-attachments/assets/eefc062b-75e9-4be4-bac1-1a474d1b437d)

### Website

The project architecture is as follows:

The site is connected to Azure using the server.js file, where the Event Hub settings (connection strings) are defined. The site uses a WebSocket server on port 3000 to enable Azure connectivity.

The ChatBox.vue file is a component embedded inside the RobotDetail.vue component, which communicates using WebSocket to Azure Event Hub. When a message arrives from Azure Event Hub, it appears in the chatbox as:
```
Partition: <partition number between 1-4>
Group: $Default
Message: "<message content>"
Status: Message Received
```
If you send a message/command through the text field to the Azure Event Hub, the message will be shown in the chatbox as:

```
Message: "<message content>"
Status: Message Sent
```

**Important note!**

You can't send commands to the robot (yet) with the text field. This needs more configuration in the robot's side.

### Azure Stream Analytics Job

For the connection to work, you need to define the bot in Azure and ensure that the Azure connections are defined correctly in the **Azure Stream Analytics Job** (name: **iot-data-analytics**). Instructions on how to add a robot to Azure can be found in [Azure Devices](#azure-devices).

Navigate to the Stream Analytics Job (**iot-data-analytics**) tab and select the "**Query**" section from the left navigation bar. Here you can see the current configuration.

![eventhub](https://github.com/user-attachments/assets/007ee870-129f-4275-995d-c166408be4d1)

### The Robot (Jetson Orin Nano, Snower)

Jetson Orin Nano uses a file called **zed2_image_publisher.py**, which can be found at:

```
cd ros2_ws/src/zed2_image_publisher/zed2_image_publisher/zed2_image_publisher.py
```
This file creates a ROS2 node which subscribes to the ZED 2 camera topic **"/zed/zed_node/left/image_rect_color"**, converts them to PILImage format, resizes the file if necessary (if the file size is too large) and converts them to a base64 string, after which the files are sent to Azure Iot Hub using the connection string.

### ZED 2 Camera

For the camera connection to work, check that the camera is connected to the Jetson Ori Nano with a USB cable and run the following commands:

```
source ~/.bashrc
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```
Sometimes the camera will show the error message "Camera not detected." When this happens, you can unplug the USB and put it back (while the program ISN'T running) and then try the solution below.
If you don't want to connect the camera to ROS 2 but want to check if the camera works, navigate to this location:

```
cd /usr/local/zed/tools/
```

And run the command:
```
/.ZED_Explorer
```
