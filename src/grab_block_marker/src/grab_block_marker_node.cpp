#include "grab_block_marker.h"


int main(int argc, char** argv) {
	ros::init(argc, argv, "grab_block_marker");

	GrabBlockMarker marker{argc, argv};
	ros::Rate loop_rate(100);
	while (ros::ok()) {
		marker.publish();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
