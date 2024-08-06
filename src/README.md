# Visual Odometry Algorithm Based on Optical Flow

## Overview
This project develops a robust visual odometry algorithm leveraging optical flow theory and data fusion techniques within the ROS2 framework. It estimates the trajectory of a moving platform by analyzing sequences of images from an onboard camera directed to the ground and data from an Inertial Measurement Unit (IMU). The fusion of these data sources is managed through a Kalman filter, ensuring enhanced accuracy and reliability of the trajectory estimates in a ROS2 environment.
- This package is exclusively built for ROS2. It is being tested on Ubuntu 22.04 with ROS2-Humble.

## Key Features
- **Optical Flow Utilization:** Utilizes optical flow algorithms to compute motion between successive camera frames, providing detailed motion vectors that contribute to trajectory estimation.
- **Data Fusion with Kalman Filter:** Employs a Kalman filter to effectively fuse visual data from the camera and motion data from the IMU. This approach mitigates errors inherent in each sensor, leading to more precise motion tracking.
- **ROS2 Integration:** Fully integrated with the ROS2 ecosystem, facilitating easy deployment and scalability in robotic applications.
- **Robust to Various Conditions:** Designed to perform under varying lighting and environmental conditions, ensuring reliable operation in both indoor and outdoor scenarios.
- **Real-Time Processing Capabilities:** Optimized for real-time performance, suitable for applications requiring immediate feedback such as autonomous vehicles and robotic navigation.

## Prerequisites
- **ROS2 Installation:** This project requires a working installation of ROS2. Follow the official ROS2 installation guide [here](https://docs.ros.org/en/rolling/Installation.html).
- **Python 3.x:** Ensure Python 3.7 or newer is installed.
- **Camera and IMU Hardware:** Compatible camera and IMU sensors properly configured for use with ROS2.

## Installation
1. Install ROS2 and necessary tools:
    ```bash
    source /opt/ros/<ros2-distro>/setup.bash
    sudo apt update
    sudo apt install python3-colcon-common-extensions
    ```
2. Set up your ROS2 workspace:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone [https://github.com/Asaf2445/visual_odometry_project.git]
    cd ..
    colcon build
    source install/setup.bash
    ```
3. Install additional ROS packages:
    ```bash
    sudo apt install ros-<ros2-distro>-usb-cam
    sudo apt install ros-<ros2-distro>-opencv
    ```
4. Install Python dependencies:
    ```bash
    pip install -r requirements.txt
    ```

## Usage
To run the visual odometry algorithm within the ROS2 framework:
    ```bash
    ros2 launch project_bringup optical_flow_app.launch.py 
    ```
Ensure that the camera and IMU are properly configured and connected as specified in the setup documentation.

## Contributing
Contributions to this project are welcome! Please refer to `CONTRIBUTING.md` for guidelines on how to make contributions.

## License
This project is licensed under the MIT License - see the `LICENSE` file for details.

## Contact
For questions and feedback, please contact [your-email] or raise an issue in the project repository.
