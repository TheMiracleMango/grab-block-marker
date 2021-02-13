#include <gazebo_msgs/ModelState.h>

#include "grab_block_marker.h"

#include <limits>
#include <cmath>


GrabBlockMarker::GrabBlockMarker(int argc, char** argv) :
	pnh_(new ros::NodeHandle("~")), nh_(new ros::NodeHandle()), tfl_(tfb_) {
	pnh_->getParam("marker_name", marker_name_);
	pnh_->getParam("maximum_grab_distance", maximum_grab_distance_);

	XmlRpc::XmlRpcValue block_name_parameters;
	pnh_->param("block_names", block_name_parameters, block_name_parameters);
	std::vector<std::string> block_names;
	for(int i = 0; i < block_name_parameters.size(); i++) {
		block_names.push_back(block_name_parameters[i]);
	}

	interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("marker","",false));

	init(block_names);

	ROS_INFO("initialized GrabBlockMarker");
}


GrabBlockMarker::~GrabBlockMarker() {
	pnh_.reset();
	nh_.reset();
	interactive_marker_server_.reset();
}


void GrabBlockMarker::init(const std::vector<std::string> &block_names) {
	holding_publisher_ = nh_->advertise<grab_block_marker::holding>("holding", 100);
	gazebo_marker_state_publisher_ = nh_->advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	gazebo_block_states_subscriber_ = nh_->subscribe("gazebo/model_states", 100, &GrabBlockMarker::blockStateCallback, this);

	// Initialize poses of expected Gazebo models
	geometry_msgs::TransformStamped initial_transform;
	initial_transform.header.frame_id = "world";
	initial_transform.transform.translation.x = 0.0;
	initial_transform.transform.translation.y = 0.0;
	initial_transform.transform.translation.z = 0.0;
	initial_transform.transform.rotation.x = 0.0;
	initial_transform.transform.rotation.y = 0.0;
	initial_transform.transform.rotation.z = 0.0;
	initial_transform.transform.rotation.w = 1.0;
	for(const auto &block_name: block_names) {
		initial_transform.child_frame_id = block_name;
		blocks_[block_name] = initial_transform;
	}

	// Initialize interactive marker state
	state_.holding_state.holding = false;
	state_.holding_state.block_name = "";

	state_.pose.header.frame_id = "world";
	state_.pose.child_frame_id = marker_name_;
	state_.pose.transform.translation.x = 0.0;
	state_.pose.transform.translation.y = 0.0;
	state_.pose.transform.translation.z = 0.0;
	state_.pose.transform.rotation.x = 0.0;
	state_.pose.transform.rotation.y = 0.0;
	state_.pose.transform.rotation.z = 0.0;
	state_.pose.transform.rotation.w = 1.0;

	marker_pose_.position.x = 0.0;
	marker_pose_.position.y = 0.0;
	marker_pose_.position.z = 0.0;
	marker_pose_.orientation.x = 0.0;
	marker_pose_.orientation.y = 0.0;
	marker_pose_.orientation.z = 0.0;
	marker_pose_.orientation.w = 1.0;

	// Set grab pose relative to the interactive marker's pose
	XmlRpc::XmlRpcValue grab_pose_from_marker;
	pnh_->param("grab_pose_from_marker", grab_pose_from_marker, grab_pose_from_marker);
	state_.grab_pose.header.frame_id = marker_name_;
	state_.grab_pose.child_frame_id = "grab";
	state_.grab_pose.transform.translation.x = grab_pose_from_marker["x"];
	state_.grab_pose.transform.translation.y = grab_pose_from_marker["y"];
	state_.grab_pose.transform.translation.z = grab_pose_from_marker["z"];
	state_.grab_pose.transform.rotation.x = grab_pose_from_marker["qx"];
	state_.grab_pose.transform.rotation.y = grab_pose_from_marker["qy"];
	state_.grab_pose.transform.rotation.z = grab_pose_from_marker["qz"];
	state_.grab_pose.transform.rotation.w = grab_pose_from_marker["qw"];

	initMarker();
}


void GrabBlockMarker::initMarker() {
	// Set interactive marker pose
	geometry_msgs::Pose pose;
	pose.position.x = state_.pose.transform.translation.x;
	pose.position.y = state_.pose.transform.translation.y;
	pose.position.z = state_.pose.transform.translation.z;
	pose.orientation.x = state_.pose.transform.rotation.x;
	pose.orientation.y = state_.pose.transform.rotation.y;
	pose.orientation.z = state_.pose.transform.rotation.z;
	pose.orientation.w = state_.pose.transform.rotation.w;

	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "world";
	int_marker.header.stamp = ros::Time::now();
	int_marker.pose = pose;
	int_marker.scale = 0.5;
	int_marker.name = marker_name_;
	int_marker.description = "3-DOF control with menu";

	// Add a box visual for the interactive marker
	visualization_msgs::InteractiveMarkerControl box_control;
	box_control.always_visible = true;

	visualization_msgs::Marker box_marker;
	box_marker.type = visualization_msgs::Marker::CUBE;
	box_marker.scale.x = int_marker.scale * 0.45;
	box_marker.scale.y = int_marker.scale * 0.45;
	box_marker.scale.z = int_marker.scale * 0.45;
	box_marker.color.r = 0.5;
	box_marker.color.g = 0.5;
	box_marker.color.b = 0.5;
	box_marker.color.a = 1.0;

	box_control.markers.push_back(box_marker);
	int_marker.controls.push_back(box_control);

	// Set control of the interactive marker
	visualization_msgs::InteractiveMarkerControl control;
	control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

	// Set x axis translation
	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "translate_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	// Set y axis translation
	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.name = "translate_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	// Set z axis translation
	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "translate_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	// Add the created marker to the interactive marker server
	interactive_marker_server_->insert(int_marker);
	interactive_marker_server_->setCallback(int_marker.name, boost::bind(&GrabBlockMarker::processFeedback, this, _1));

	// Add menu to the interactive marker
	interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Interact");
	menu_handler_.insert(sub_menu_handle, "Grab", boost::bind(&GrabBlockMarker::processFeedback, this, _1));
	menu_handler_.insert(sub_menu_handle, "Release", boost::bind(&GrabBlockMarker::processFeedback, this, _1));
	menu_handler_.apply(*interactive_marker_server_, int_marker.name);

	interactive_marker_server_->applyChanges();
}


std::string GrabBlockMarker::getNearestBlock() {
	std::string nearest_block;
	double nearest_distance = std::numeric_limits<double>::max();

	// Find the Gazebo block model that is closest to the interactive marker
	// Return empty string if the nearest block is not within the grab distance
	for(const auto &block : blocks_) {
		double x = state_.pose.transform.translation.x - block.second.transform.translation.x;
		double y = state_.pose.transform.translation.y - block.second.transform.translation.y;
		double z = state_.pose.transform.translation.z - block.second.transform.translation.z;
		double distance = std::sqrt(x * x + y * y + z * z);
		if(distance <= maximum_grab_distance_ && distance < nearest_distance) {
			nearest_distance = distance;
			nearest_block = block.first;
		}
	}

	return nearest_block;
}


void GrabBlockMarker::blockStateCallback(const gazebo_msgs::ModelStates::ConstPtr& models) {
	// Update poses of the marker and blocks from Gazebo simulation
	for (int i = 0; i < models->name.size(); ++i) {
		if (models->name[i] == marker_name_) {
			state_.pose.transform.translation.x = models->pose[i].position.x;
			state_.pose.transform.translation.y = models->pose[i].position.y;
			state_.pose.transform.translation.z = models->pose[i].position.z;
			state_.pose.transform.rotation.w = models->pose[i].orientation.w;
			state_.pose.transform.rotation.x = models->pose[i].orientation.x;
			state_.pose.transform.rotation.y = models->pose[i].orientation.y;
			state_.pose.transform.rotation.z = models->pose[i].orientation.z;
		} else {
			auto it = blocks_.find(models->name[i]);
			if (it != blocks_.end()) {
				it->second.transform.translation.x = models->pose[i].position.x;
				it->second.transform.translation.y = models->pose[i].position.y;
				it->second.transform.translation.z = models->pose[i].position.z;
				it->second.transform.rotation.x = models->pose[i].orientation.x;
				it->second.transform.rotation.y = models->pose[i].orientation.y;
				it->second.transform.rotation.z = models->pose[i].orientation.z;
				it->second.transform.rotation.w = models->pose[i].orientation.w;
			}
		}
	}
}


void GrabBlockMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	switch(feedback->event_type) {
	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		switch(feedback->menu_entry_id) {
		case 2:
			if(state_.holding_state.holding) {
				ROS_WARN_STREAM("Cannot grab while holding " << state_.holding_state.block_name);
			} else {
				std::string nearest_block = getNearestBlock();
				if(!nearest_block.empty()) {
					state_.holding_state.holding = true;
					state_.holding_state.block_name = nearest_block;
					ROS_INFO_STREAM("Grabbing " << nearest_block);
				} else {
					ROS_WARN_STREAM("Block too far away to grab");
				}
			}
			break;
		case 3:
			if(!state_.holding_state.holding) {
				ROS_WARN("Cannot release while holding nothing");
			} else {
				ROS_INFO_STREAM("Releasing " << state_.holding_state.block_name);
				state_.holding_state.holding = false;
				state_.holding_state.block_name = "";
			}
			break;
		default:
			break;
		}
		break;

	case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
		marker_pose_.position.x = feedback->pose.position.x;
		marker_pose_.position.y = feedback->pose.position.y;
		marker_pose_.position.z = feedback->pose.position.z;
		marker_pose_.orientation.x = feedback->pose.orientation.x;
		marker_pose_.orientation.y = feedback->pose.orientation.y;
		marker_pose_.orientation.z = feedback->pose.orientation.z;
		marker_pose_.orientation.w = feedback->pose.orientation.w;
		break;
	}

	interactive_marker_server_->applyChanges();
}


void GrabBlockMarker::publish() {
	auto time = ros::Time::now();

	// Publish grab transform
	state_.grab_pose.header.stamp = time;
	sbr_.sendTransform(state_.grab_pose);

	// Publish Gazebo marker tf2 transform
	state_.pose.header.stamp = time;
	br_.sendTransform(state_.pose);

	// Publish Gazebo block tf2 transforms
	for(auto &block: blocks_) {
		block.second.header.stamp = time;
		br_.sendTransform(block.second);
	}

	// Publish grab/hold state
	holding_publisher_.publish(state_.holding_state);

	// Move marker in Gazebo simulation
	gazebo_msgs::ModelState marker_gazebo_state;
	marker_gazebo_state.model_name = marker_name_;
	marker_gazebo_state.pose = marker_pose_;
	marker_gazebo_state.reference_frame = std::string("world");
	gazebo_marker_state_publisher_.publish(marker_gazebo_state);

	if(state_.holding_state.holding) {
		// Get transform from grab to world
		geometry_msgs::TransformStamped holding_block_transform;
		try {
			holding_block_transform = tfb_.lookupTransform("world", "grab", ros::Time(0));
		} catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			return;
		}

		// Move holding block in Gazebo simulation
		geometry_msgs::Pose holding_block_pose;
		holding_block_pose.position.x = holding_block_transform.transform.translation.x;
		holding_block_pose.position.y = holding_block_transform.transform.translation.y;
		holding_block_pose.position.z = holding_block_transform.transform.translation.z;
		holding_block_pose.orientation.x = holding_block_transform.transform.rotation.x;
		holding_block_pose.orientation.y = holding_block_transform.transform.rotation.y;
		holding_block_pose.orientation.z = holding_block_transform.transform.rotation.z;
		holding_block_pose.orientation.w = holding_block_transform.transform.rotation.w;

		gazebo_msgs::ModelState holding_block_gazebo_state;
		holding_block_gazebo_state.model_name = state_.holding_state.block_name;
		holding_block_gazebo_state.pose = holding_block_pose;
		holding_block_gazebo_state.reference_frame = std::string("world");
		gazebo_marker_state_publisher_.publish(holding_block_gazebo_state);
	}
}
