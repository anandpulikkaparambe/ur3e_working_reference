
# UR3e Lego Sorting with YOLOv8 (ROS 2 Jazzy)

This project implements an autonomous object detection, sorting, and pick-and-place system using a **Universal Robots UR3e** arm and **YOLOv8** perception in a Gazebo Harmonic simulation.



## 🚀 Features

*   **Advanced Perception**: Custom-trained **YOLOv8** model to detect colored Lego bricks (Red, Blue, Yellow, Green).
*   **Robust Filtering**:
    *   **ROI (Region of Interest)**: Dynamically configurable Search Zones to exclude robot parts and background noise.
    *   **Priority Logic**: Specifically targets "Red" bricks first (User Configurable).
*   **Simulation**: High-fidelity **Gazebo Harmonic** simulation with physics interactions, conveyor belts, and sorting bins.
*   **Motion Planning**: Integrated **MoveIt 2** for collision-free path planning and execution.
*   **Gripper Control**: Simulates a **Robotiq 2F-140** gripper.

## 🛠️ Tech Stack

*   **OS**: Ubuntu 24.04 LTS (Noble)
*   **Middleware**: ROS 2 Jazzy
*   **Simulation**: Gazebo Harmonic (LTS until September 2028)
*   **Planning**: MoveIt 2
*   **Vision**: Ultralytics YOLOv8 + OpenCV
*   **Language**: Python 3.12, C++

## 📦 Installation

### Prerequisites

Ensure you have **Ubuntu 24.04** and **ROS 2 Jazzy** installed. Verify your installation:

```bash
# Verify Ubuntu version
lsb_release -a  # Should show Ubuntu 24.04

# Verify ROS 2 Jazzy
printenv ROS_DISTRO  # Should output: jazzy
```

If you haven't installed ROS 2 Jazzy yet, follow the [official installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). [docs.ros](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### 1. Install Gazebo Harmonic for ROS 2 Jazzy

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Gazebo Harmonic (recommended for Jazzy)
sudo apt install ros-jazzy-ros-gz

# Verify Gazebo installation
gz sim --version  # Should show Gazebo Harmonic version
```

### 2. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/your-username/ur3e_lego_sorting.git
cd ~/ros2_ws
```

### 3. Install Dependencies

```bash
# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install Python packages for YOLOv8
pip install ultralytics opencv-python
```

### 4. Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 🖥️ Usage

This system is designed to run in **3 separate terminals**. Make sure to source your workspace in each terminal:

```bash
source ~/ros2_ws/install/setup.bash
```

### Terminal 1: Simulation (Gazebo Harmonic + MoveIt)
Launches the robot, world, and controllers.

```bash
ros2 launch ur_gazebo ur_gazebo.launch.py \
    ur_type:=ur3e \
    launch_rviz:=false \
    world_file:=pick_and_place_demo.world
```

*Note: For Jazzy, the launch file structure may differ slightly from Humble. Ensure your launch files use the `ros_gz_bridge` instead of `gazebo_ros_pkgs`.

### Terminal 2: Perception (YOLO Detector)
Runs the computer vision node. It opens a "YOLO Detection" window showing the camera feed and bounding boxes.

```bash
ros2 run ur3e_sorting yolo_detector
```

*   **Green/Blue Boxes**: Valid detections.
*   **Search Zone**: The area where the robot visualizes/looks.

### Terminal 3: Logic (Sorting Manager)
The brain of the operation. Subscribes to detections and commands the robot.

```bash
ros2 run ur3e_sorting sorting_node
```

## ⚙️ Configuration & Tuning

### Gazebo Harmonic Specific Changes

**Important**: Gazebo Harmonic uses a different plugin system and bridge compared to Gazebo Classic.

*   **Camera Plugin**: Use `<sensor type="camera">` with `<gz-sim>` tags instead of `libgazebo_ros_camera.so`
*   **Bridge Topics**: Use `ros_gz_bridge` to connect Gazebo topics to ROS 2:
    ```bash
    ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
    ```

### Camera Configuration

*   **Position**: Defined in `src/ur_gazebo/worlds/pick_and_place_demo.world`
    *   *Current*: `<pose>0.3 0.0 1.5 0 0 0</pose>` (Zoomed in for accuracy)
*   **Camera Plugin (Harmonic format)**:
    ```xml
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
    </sensor>
    ```

### Detection Filters

Configured in `src/ur3e_sorting/ur3e_sorting/yolo_detector.py`:

*   **ROI**: `Y < 380` (Top Strip) to exclude robot base
*   **Confidence**: `0.05` (High Sensitivity)
*   **Image Topic**: Adjust for Jazzy's topic remapping if needed

### Pick & Place Logic

`src/ur3e_sorting/ur3e_sorting/sorting_node.py`:

*   **Priority**: Prioritizes Red Bricks over others
*   **Safe Z**: Plans picks 2cm above detection to avoid table collisions
*   **MoveIt 2 Interface**: Updated for Jazzy's MoveGroup API

## 📂 Directory Structure

```
.
├── src
│   ├── ur3e_sorting       # Main logic (YOLO, Sorting Node, Launch files)
│   ├── ur_gazebo          # Gazebo Harmonic worlds and integration
│   ├── ur_description     # URDFs and meshes (Updated for Jazzy compatibility)
│   └── moveit_config      # MoveIt 2 configuration packages (Jazzy-compatible)
├── YoloV8_v2              # Trained weights and model data
└── README.md
```

## 🔄 Migration from Humble to Jazzy

Key changes made for Jazzy compatibility:

1. **Gazebo Classic → Gazebo Harmonic** [youtube](https://www.youtube.com/watch?v=dY8JxldcuqA)
   - Replaced all `gazebo_ros` plugins with `ros_gz` equivalents
   - Updated SDF world files to Harmonic format
   - Modified camera and sensor plugins

2. **Launch Files**
   - Updated to use `ros2 launch` with Jazzy conventions
   - Changed `gazebo_ros_pkgs` to `ros_gz_sim`

3. **Python Updates**
   - Updated for Python 3.12 compatibility
   - Modified ROS 2 node interfaces for Jazzy API changes

4. **MoveIt 2 Configuration** [automaticaddison](https://automaticaddison.com/configure-moveit-2-for-a-simulated-robot-arm-ros-2-jazzy/)
   - Regenerated MoveIt configs using Jazzy's setup assistant
   - Updated `moveit_servo` parameters if used
   - Adjusted controller configurations for Jazzy's `ros2_control`

## 🐛 Troubleshooting

### Jazzy-Specific Issues

*   **Gazebo Won't Launch?**: Verify Gazebo Harmonic installation:
    ```bash
    gz sim --version  # Should show Harmonic
    ros2 pkg list | grep ros_gz  # Should show ros_gz packages
    ```

*   **Camera Not Publishing?**: Gazebo Harmonic uses different topic names. Check with:
    ```bash
    ros2 topic list | grep camera
    # Use ros_gz_bridge if needed to remap topics
    ```

*   **MoveIt Planning Fails?**: Jazzy has stricter collision checking. Try:
    - Increase planning time in `moveit_config`
    - Adjust joint limits in `moveit_configs/config/joint_limits.yaml`
    - Use relaxed orientation constraints in your sorting node [automaticaddison](https://automaticaddison.com/configure-moveit-2-for-a-simulated-robot-arm-ros-2-jazzy/)

### General Issues

*   **Robot Floating?**: Ensure URDF has correct `<pose>0 0 0.8 0 0 0</pose>` and spawn height is `0.0`
*   **Move Failed?**: Often due to strict orientation constraints. The `sorting_node` uses relaxed tolerances to find valid IK solutions
*   **Camera Blocked?**: Adjust the `ROI` inside `yolo_detector.py` to mask out the robot body if it enters the frame

## 📚 Additional Resources

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/) [docs.ros](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Gazebo Harmonic with ROS 2](https://gazebosim.org/docs/latest/ros_installation/) [gazebosim](https://gazebosim.org/docs/latest/ros_installation/)
- [MoveIt 2 for Jazzy](https://moveit.picknik.ai/jazzy/index.html) [automaticaddison](https://automaticaddison.com/configure-moveit-2-for-a-simulated-robot-arm-ros-2-jazzy/)
- [UR Robots with ROS 2 Jazzy](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) [5qu1d1nk](https://5qu1d1nk.art/ros2-jazzy-with-ur-robots-and-moveit/)

## 🤝 Contributing

Contributions are welcome! Please ensure compatibility with:
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

## 📝 License

Your License Here - any issues please contact me - anandpsdit@gmail.com



