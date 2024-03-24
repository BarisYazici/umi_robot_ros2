# umi_robot_ros2

This project provides:

- ROS 2 Hardware interface,
- ROS 2 UMI Robot MoveIt package,
- and MicroROS ESP32 component for Servo control with a serial connection.

## Getting Started

### Hardware

This project uses the UMI robot hardware designed by Murilo Marques Marinho, Ph.D. It's a low-cost 3D printable robotic platform. You can access the UMIRobot website [here](https://mmmarinho.github.io/UMIRobot/). UMIRobot is an excellent platform for both robotics enthusiasts and students.


### MicroROS Setup

MicroROS is a platform designed to bring ROS2 (Robot Operating System 2) to microcontrollers. This project uses a component for ESP-IDF (Espressif IoT Development Framework) that integrates MicroROS.

To set up MicroROS for this project, follow these steps:

1. Clone the MicroROS for ESP-IDF component repository:

```bash
git clone https://github.com/BarisYazici/micro_ros_espidf_component.git
```

2. Start the repository using the devcontainer for VSCode.

  - Open the project in Visual Studio Code.
  - Move the `.devcontainer` directory to the top level of your project.
  - Press `F1` to open the command palette, then run the `Dev Containers: Rebuild container` command.


3. Source the ESP-IDF environment setup script. This script sets up the environment variables and paths necessary for the ESP-IDF. Depending on the shell you're using, the command might be:

```bash
source $IDF_PATH/export.sh
```

4. Switch to the servo_drive example folder.

```bash
cd examples/servo_drive
```

5. Build the example servo_drive project

```bash
idf.py build
```

6. Connect the ESP32 to your laptop. And give read-write permission to the docker container user.

```bash
sudo chmod a+rw /dev/ttyUSB0
```

7. Flash the ESP32 with the servo_drive example

```bash
idf.py flash
```

### Setting up the umi_robot_ros2 workspace

This project uses a development container in Visual Studio Code for a consistent and easily reproducible development environment.

1. Ensure you have [Docker](https://www.docker.com/get-started) installed and running on your machine.
2. Install [Visual Studio Code](https://code.visualstudio.com/download).
3. Install the [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension in Visual Studio Code.
4. Open the project in Visual Studio Code.
5. Move the `.devcontainer` directory to the top level of your project.
6. Press `F1` to open the command palette, then run the `Dev Containers: Rebuild container` command.

This will build the Docker image as defined in the `.devcontainer` directory and start a container. Your workspace will automatically be mounted into the container, and you'll be connected to the running container.

### Creating a ROS2 Workspace

Once you're in the development container, you can create a ROS2 workspace:

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Building the Project

To build the project,

```sh

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

```

### Connecting the ROS 2 Agent

Make sure that you connected the ESP32 through serial communication to your computer.
Run the following docker container to connect the MicroROS agent.

```
docker run -it --rm --device=/dev/ttyUSB0 microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200 -v6
```

### Running the MoveIt Example

After connecting the ROS 2 agent making sure that you can receive data from the Microros agent in ESP32 microcontroller, you can run the MoveIt example with the following command:

```sh
ros2 launch umi_robot_moveit_config demo.launch.py
```

This will start the RViz2 application with MoveIt motion plannning plugin.
