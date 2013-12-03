#include "ros/ros.h"
#include "heartbeat/HeartbeatClient.h"

#include <stdlib.h>

int main(int argc, char **argv) {
	heartbeat::State state;
	heartbeat::State req_state;
	bool success;

	ros::init(argc, argv, "heartbeat_client");

	ros::NodeHandle n;
	ros::Rate loop_rate(0.5);

	HeartbeatClient hb(n);
	hb.start();

	while (ros::ok()) {
		state = hb.getState();
		ROS_INFO("Current: %d", state.value);

		req_state.value = rand() % 4;
		success = hb.setState(req_state);

		ROS_INFO("setState (%u -> %u): %u", state.value,
				req_state.value, success);

//		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
