# UR3e Lego Sorting with YOLOv8 (ROS 2 jazzy)

This project implements an autonomous object detection, sorting, and pick-and-place system using a **Universal Robots UR3e** arm and **YOLOv8** perception in a Gazebo simulation.

![System Overview](https://raw.githubusercontent.com/placeholder/image.png)

## 🚀 Features

*   **Advanced Perception**: Custom-trained **YOLOv8** model to detect colored Lego bricks (Red, Blue, Yellow, Green).
*   **Robust Filtering**:
    *   **ROI (Region of Interest)**: Dynamically configurable Search Zones to exclude robot parts and background noise.
    *   **Priority Logic**: Specifically targets "Red" bricks first (User Configurable).
*   **Simulation**: High-fidelity Gazebo simulation with physics interactions, conveyor belts, and sorting bins.
*   **Motion Planning**: Integrated **MoveIt 2** for collision-free path planning and execution.
*   **Gripper Control**: Simulates a **Robotiq 2F-140** gripper.

## 🛠️ Tech Stack

*   **OS**: Ubuntu 22.04 LTS (Jammy)
*   **Middleware**: ROS 2 Humble
*   **Simulation**: Gazebo
*   **Planning**: MoveIt 2
*   **Vision**: Ultralytics YOLOv8 + OpenCV
*   **Language**: Python 3.10, C++

## 📦 Installation

1.  **Clone the Repository**:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/your-username/ur3e_lego_sorting.git
    cd ~/ros2_ws
    ```

2.  **Install Dependencies**:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    pip install ultralytics opencv-python
    ```

3.  **Build the Workspace**:
    ```bash
    colcon build
    source install/setup.bash
    ```

## 🖥️ Usage

This system is designed to run in **3 separate terminals**.

### Terminal 1: Simulation (Gazebo + MoveIt)
Launches the robot, world, and controllers.
```bash
ros2 launch ur_gazebo ur.gazebo.launch.py ur_type:=ur3e world_file:=pick_and_place_demo.world
```
*Note: Ensure the robot spawns firmly on the table. If it floats, check the launch arguments.*

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

*   **Camera Position**: Defined in `src/ur_gazebo/worlds/pick_and_place_demo.world`.
    *   *Current*: `0.3 0.0 1.5` (Zoomed in for accuracy).
*   **Detection Filters**: Configured in `src/ur3e_sorting/ur3e_sorting/yolo_detector.py`.
    *   **ROI**: `Y < 380` (Top Strip) to exclude robot base.
    *   **Confidence**: `0.05` (High Sensitivity).
*   **Pick Logic**: `src/ur3e_sorting/ur3e_sorting/sorting_node.py`
    *   **Priority**: Prioritizes Red Bricks over others.
    *   **Safe Z**: Plans picks 2cm above detection to avoid table collisions.

## 📂 Directory Structure

```
.
├── src
│   ├── ur3e_sorting       # Main logic (YOLO, Sorting Node, Launch files)
│   ├── ur_gazebo          # Simulation worlds and Gazebo integration
│   ├── ur_description     # URDFs and meshes (Modified for Z-fixes)
│   └── moveit_config      # MoveIt configuration packages
├── YoloV8_v2              # Trained weights and model data
└── README.md
```

## 🐛 Troubleshooting

*   **Robot Floating?**: We fixed a "Double Elevation Bug". Ensure `ur_robotiq_140.urdf.xacro` has the correct `0.8m` offset and the launch file spawns at `0.0`.
*   **Move Failed?**: Often due to strict orientation constraints. The `sorting_node` uses relaxed tolerances to find valid IK solutions.
*   **Camera Blocked?**: Adjust the `ROI` inside `yolo_detector.py` to mask out the robot body if it enters the frame.
