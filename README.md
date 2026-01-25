
# ROS 2 Camera-Based Robot Control Interface (Click & ArUco Teleoperation)

## Project Description
**ROS2_camnode_controller_interface** is a vision-based robot control system developed using ROS 2 Humble, OpenCV, and Docker. The project replaces traditional keyboard or joystick teleoperation with an intuitive camera-based interface, allowing a user to control a mobile robot by either:  

- Clicking on a live camera feed  
- Moving an ArUco marker in front of the camera  

The system processes real-time camera images, interprets user interaction (mouse clicks or ArUco marker position), and publishes motion commands to the robot via ROS 2 topics.  

The project supports:  
- Mobile robot control using `/turtle1/cmd_vel`  
- Local execution
- Dockerized execution   
- Single-command launch files  
- TurtleSim   

---

## Features

### Core Features
- 📷 Live camera feed using `v4l2_camera`  
- 🖱️ Mouse click teleoperation  
- 🏷️ ArUco marker-based control  
- 🤖 Mobile robot motion via `//turtle1/cmd_vel`  
- 🧠 Custom ROS 2 Python nodes  
- 🚀 Single launch file execution  
- 🧭 TurtleSim visualization  

---

## 3. Repository Structure
```
ROS2_camnode_controller_interface/
├── Dockerfile
├── docker-compose.yml
├── README.md
└── ros2_ws/
    └── src/
        └── camera_click_teleop/
            ├── camera_click_teleop/
            │   ├── __init__.py
            │   ├── click_teleop_node.py
            │   └── aruco_teleop_node.py
            ├── launch/
            │   ├── click_teleop.launch.py
            │   └── aruco_teleop.launch.py
            ├── package.xml
            ├── setup.py
            └── setup.cfg
```

---

## System Architecture

### Data Flow
```
[Webcam]
   │
   ▼ /image_raw
[v4l2_camera_node]
   │
   ▼ cv_bridge
[Click / ArUco Teleop Node]
   │
   ▼ Motion Logic
   │
   ▼ /cmd_vel
[Mobile Robot Simulation]
   │
   ▼ [TurtleSim Visualization]
```

---

## Dependencies

**Tested On:** Ubuntu 22.04  
**ROS 2 Version:** Humble 

### Required Packages (Local)
```bash
sudo apt update
sudo apt install ros-humble-desktop \  
                 ros-humble-v4l2-camera \  
                 ros-humble-cv-bridge \  
                 ros-humble-turtlesim  \ 
                 ros-humble-rviz2  \ 
                 pip3 install "numpy<2.0"\
                 pip3 install "opencv-contrib-python==4.10.0.84" \
                 python3-opencv
```

### Docker Requirements
- Docker Engine  
- Docker Compose  
- X11 display support  
- Webcam (built-in or USB)  

---

## Build Instructions (Local)
```bash
cd ~/ros2_ws/
colcon build --packages-select camera_click_teleop
source install/setup.bash
```

---

## How to Run (Local)

### ArUco Teleoperation
```bash
ros2 launch camera_click_teleop aruco_teleop.launch.py
```

### Click Teleoperation
```bash
ros2 launch camera_click_teleop click_teleop_with_v4l2.launch.py
```

---

## How to Run (Docker)

### Step 1: Allow Docker GUI Access (Host)
```bash
xhost +local:docker
docker-compose up -d

```

### Step 2: Build Docker Image
```bash
docker build -t ros2_camnode .
```

### Step 3: Run Container
```bash
docker run -it --rm  \ 
           --net=host \  
           --device=/dev/video0 \  
           -e DISPLAY=$DISPLAY \  
           -v /tmp/.X11-unix:/tmp/.X11-unix \  
           ros2_camnode
```

### Step 4: Launch Application (Inside Container)
```bash
ros2 launch camera_click_teleop aruco_teleop.launch.py
```

---

## Interface Behavior

### ArUco Mode
- 📈 Marker above image center → Forward motion  
- 📉 Marker below image center → Backward motion  
- 🚫 Marker lost → Stop  

### Click Mode
- 🖱️ Click above center → Forward  
- 🖱️ Click below center → Backward  

**Overlay graphics display:**  
- Image center reference line  
- Marker bounding box or click indicator  
- Status text  

---

## Debugging & Verification

### Verify Camera
```bash
ros2 topic echo /image_raw --once
```

### Verify Velocity Commands
```bash
ros2 topic echo /turtle1/cmd_vel
```

### Verify Nodes
```bash
ros2 node list
```

### Common Issues
- **No camera image:** Ensure `/dev/video0` is passed to Docker  
- **GUI not visible:** Check `$DISPLAY` and X11 permissions  
- **Package not found:** Run `source install/setup.bash`  

---

## Demo
A demonstration video is included in the repository showing:  
- Camera feed  
- ArUco detection  
- Robot motion  
- RViz visualization  

---

## Authors
**[Azeez Oladipupo Akinlade]**  
**[Haithem Ladj]**

---

## License
This project was created for an academic ROS2 project and is intended for **educational use**.
