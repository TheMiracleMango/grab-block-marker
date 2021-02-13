#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <gazebo_msgs/ModelStates.h>

#include "grab_block_marker/holding.h"

#include <memory>
#include <vector>
#include <map>
#include <string>


class GrabBlockMarker {
public:
GrabBlockMarker(int argc, char** argv);
~GrabBlockMarker();

// Publish tf2, Gazebo model states, and holding status
void publish();

private:
struct State {
	geometry_msgs::TransformStamped pose;
	geometry_msgs::TransformStamped grab_pose;
	grab_block_marker::holding holding_state;
};

void init(const std::vector<std::string> &block_names);

// Initialize interactive marker
void initMarker();

// Interactive marker callback
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

// Gazebo model states callback
void blockStateCallback(const gazebo_msgs::ModelStates::ConstPtr& block);

// Return the name of the Gazebo model nearest to the interactive marker
// Return empty string if the block is further than the grab distance
std::string getNearestBlock();

tf2_ros::TransformBroadcaster br_;
tf2_ros::StaticTransformBroadcaster sbr_;
tf2_ros::Buffer tfb_;
tf2_ros::TransformListener tfl_;

std::unique_ptr<ros::NodeHandle> pnh_;
std::unique_ptr<ros::NodeHandle> nh_;
ros::Publisher holding_publisher_;
ros::Publisher gazebo_marker_state_publisher_;
ros::Subscriber gazebo_block_states_subscriber_;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
interactive_markers::MenuHandler menu_handler_;

std::map<std::string, geometry_msgs::TransformStamped> blocks_;
State state_;

// Keep tf2 transform from Gazebo and marker pose in Rviz separated
geometry_msgs::Pose marker_pose_;

std::string marker_name_;
double maximum_grab_distance_;
};
