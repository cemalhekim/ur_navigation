#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>

class Navigation : public rclcpp::Node
{
public:
    Navigation()
    : Node("navigation")
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("ur_manipulator");
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>("base_link", "moveit_visual_tools");
        visual_tools_->deleteAllMarkers();
        visual_tools_->loadRemoteControl();
        visual_tools_->trigger();

        subscription_random_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "random_coordinates", 10, std::bind(&Navigation::listener_callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Navigation::process_target, this));
        
        target_reached_ = true; // Initially set to true
    }

private:
    void listener_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        target_pose_ = *msg;
        target_reached_ = false;
        RCLCPP_INFO(this->get_logger(), "Moving to target location: (%f, %f, %f)", 
                    target_pose_.position.x, target_pose_.position.y, target_pose_.position.z);
    }

    void process_target()
    {
        if (!target_reached_)
        {
            move_group_->setPoseTarget(target_pose_);
            
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            
            if (success)
            {
                move_group_->execute(plan);
                target_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "Reached target position.");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to plan to target position.");
            }
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_random_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    
    geometry_msgs::msg::Pose target_pose_;
    bool target_reached_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Navigation>());
    rclcpp::shutdown();
    return 0;
}

------------------------------------------------------------------------------------------


















cmake_minimum_required(VERSION 3.5)
project(navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${moveit_ros_planning_interface_INCLUDE_DIRS}
    ${moveit_INCLUDE_DIRS}
    ${moveit_visual_tools_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
)

# Add the executable for the navigation node
add_executable(harvest_location /home/ch/harvester/ros2_ws/src/navigation/navigation/harvest_location.cpp)
ament_target_dependencies(harvest_location rclcpp geometry_msgs)

# Add the executable for the TurtleSim control node
add_executable(turtlesimcontrol /home/ch/harvester/ros2_ws/src/navigation/navigation/turtlesimcontrol.cpp)
ament_target_dependencies(turtlesimcontrol rclcpp geometry_msgs turtlesim)

# Add the executable for the TurtleSim control node
add_executable(navigation /home/ch/harvester/ros2_ws/src/navigation/navigation/navigation.cpp)
ament_target_dependencies(navigation
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
    moveit
    moveit_visual_tools
    tf2
    tf2_ros
)

install(TARGETS
  harvest_location
  turtlesimcontrol
  navigation
  DESTINATION lib/${PROJECT_NAME})

ament_package()
