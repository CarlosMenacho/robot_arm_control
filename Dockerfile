FROM osrf/ros:jazzy-desktop-full
    
    RUN apt-get update && apt-get install -y \
        ros-jazzy-moveit \
        ros-jazzy-moveit-setup-assistant \
        ros-jazzy-moveit-configs-utils \
        ros-jazzy-moveit-ros-planning \
        ros-jazzy-moveit-ros-move-group \
        ros-jazzy-moveit-ros-visualization \
        ros-jazzy-moveit-kinematics \
        ros-jazzy-moveit-planners-ompl \
        ros-jazzy-ros2-control \
        ros-jazzy-controller-manager \
        ros-jazzy-ros2-controllers \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-rmw-cyclonedds-cpp \
        && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=20

RUN echo "source /opt/ros/jazzy/setup.bash" >>/root/.bashrc

# moveit takes alphanumerical order when sending to /joint_states

# some findings:
    # do not name moveit controller group as any of the joint in the robot 
    # add allways limit velocity and accel to moveit cfg controller
    # All joint limits must be in decimals
    