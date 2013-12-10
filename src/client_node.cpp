#include "ros/ros.h"
#include "heartbeat/HeartbeatClient.h"

#include <stdlib.h>

int main(int argc, char **argv) {
	heartbeat::State::_value_type state;
	heartbeat::State::_value_type req_state;
	int cnt = 100;
	bool success;

	ros::init(argc, argv, "heartbeat_client", ros::init_options::AnonymousName);

	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	HeartbeatClient hb(n, 0.1);
	hb.start();

	while (--cnt > 0) {
		hb.alive();
		state = hb.getState();
		ROS_INFO("Current: %d", state);

		req_state = rand() % 5;
		success = hb.setState(req_state);

		ROS_INFO("setState (%u -> %u): %u", state,
				req_state, success);

//		ros::spinOnce();
 		loop_rate.sleep();
	}

	return 0;
}
