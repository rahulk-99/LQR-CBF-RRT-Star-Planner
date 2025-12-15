🏁 LQR–CBF–RRT* Path Planning on F1 Track

This demonstrates safe and optimal path planning on a Formula 1 race track using the LQR–CBF–RRT* algorithm. The implementation includes a 2D planner and 3D validation with TurtleBot3 in Gazebo via ROS 2. With a larger F1 map, the code has been optimized to achieve a runtime of 7 minutes. A screen-recorded video demonstrating the improvements is available via the drive link at the bottom of the document.
----------------------------------------
📂 Folder Structure

planning_submission/
├── LQR_CBF_rrtStar_planning/        # Python planner using LQR + CBF + RRT*
├── ros2_package_f1_track/           # ROS 2 package for Gazebo validation
├── LQR-CBF_result.PNG               # Sample result image
└── README.txt                       # This file

----------------------------------------
🔧 Install Dependencies

Before running the planner, install the required Python packages:

pip3 install -r requirements.txt

----------------------------------------
🚀 How to Run the Project

1️⃣ Run the LQR–CBF–RRT* Planner

Note:  Make sure you're in the root directory of the project
cd /path/to/final_code_ros_package

python3 LQR_CBF_rrtStar_planning/linear_dynamic_model/LQR_CBF_rrtStar_linear.py

✔ This will:
- Load the 2D map of the Barcelona F1 track
- Plan a safe and optimal path using LQR–CBF–RRT*
- Save the generated waypoints to the ROS 2 package

----------------------------------------
2️⃣ Validate in Gazebo (ROS 2)

cd ros2_package_f1_track

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build the package
colcon build

# Source the local setup
source install/setup.bash

➡ Launch the F1 track world in Gazebo:

ros2 launch turtlebot3_project3 competition_world.launch.py

➡ In a new terminal, run the path-following node:

ros2 run turtlebot_path_follower track_follower

----------------------------------------
✅ Test Case

- Track: Circuit de Barcelona–Catalunya
- Goal: Navigate from start to finish using LQR-optimal and CBF-safe path
- Output: Smooth and safe trajectory executed in Gazebo

----------------------------------------

🖼 Sample Output

Refer to: LQR-CBF_result.PNG
Waypoints: Saved in ros2_package_f1_track/src/turtlebot_path_follower/data

----------------------------------------
💻 Performance Notes

- For PC specs with i7, 13th Gen, the runtime is approximately 7-8 minutes.
- Please keep the laptop charging for better and faster performance.


----------------------------------------
✅ Test Case

- Track: Circuit de Barcelona–Catalunya
- Goal: Navigate from start to finish using LQR-optimal and CBF-safe path
- Output: Smooth and safe trajectory executed in Gazebo

Test Case Details:
    x_start = (254, -108)  # Starting node
    x_goal  = (200, -108)  # Goal node

Note: This is the maximum distance in the track for the path generation.
You can change these values in the LQR_CBF_rrtstar_linear.py file if you want to try a different configuration.

----------------------------------------


🔧 Troubleshooting Module Detection

Some error issue:  
If not able to detect a module while running LQR_CBF_rrtStar_planning.py, try adding the following after line 12 in the same .py file:

    sys.path.append(os.path.abspath(os.path.join(__file__, '..', '..')))

----------------------------------------

🔗 Additional Resources

For Gazebo simulation videos, complete code, and the ROS package, visit:
https://drive.google.com/drive/folders/12aKWb8TTLxd9E-hk6cPMj8ICoBYIsaOZ?usp=drive_link

----------------------------------------
