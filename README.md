# Butler Robot Simulation

This repository contains software packages for a differential-drive Husky robot designed to automate food delivery in a cafe environment. The robot collects food from the kitchen and delivers it to customer's tables, improving efficiency and reducing labor costs. After completing deliveries, it returns to its home position, ready for the next task. The system is built using ROS1 Noetic, SLAM, and move_base for autonomous navigation

## ROS Workspace Setup

1. ``` mkdir -p butler_robot_ws ```
2. ``` cd butler_robot_ws ```
3. ``` git clone https://github.com/KoushikRaghav/butler-robot-sim.git src ```
4. ``` rosdep install --from-paths src --ignore-src -r -y ```
5. ``` catkin build ```

## Other dependencies

1. ``` sudo apt install tmux tmuxp -y ```
2. ``` echo alias tmuxp='export LANG=en_IN.utf8; tmuxp' >> ~/.bashrc  ```
   
## Steps to run

1. ``` cd butler_robot_ws ```
2. ``` source devel/setup.bash ```
3. ``` tmuxp load src/butler_robot/tools/butler_sim.yaml ```
4. The tmux window will be up and running and the robot will be spawned in a simulation environment
5. The user has to enter inputs in the third pane of the tmux window to provide tasks for the robot
6. To add/remove additional instructions to the robot, the user can provide data when the robot is navigating towards kitchen. For example, the user can add or remove tables for delivery. If there are no orders in the queue, the robot will execute return to home and be in idle state
7. To cancel a particular operation, the user can trigger "ctrl+c" signal which cancels the current operation and the robot executes return to kitchen/home based on the current task
