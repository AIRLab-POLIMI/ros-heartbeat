#include "ros/ros.h"
#include "heartbeat/HeartbeatClient.h"

#include <stdlib.h>

int main(int argc, char **argv) {
	heartbeat::State::_value_type state;
	heartbeat::State::_value_type req_state;
	int cnt = 1000;
	bool success;

	state = heartbeat::State::HALT;

	srand((time(NULL) & 0xFFFF) | (getpid() << 16));

	ros::init(argc, argv, "heartbeat_dumb", ros::init_options::AnonymousName);

	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	HeartbeatClient hb(n);
	hb.start();

	while (--cnt > 0) {
		if (cnt % 1 == 0) {
			state = hb.getState();
			ROS_INFO("Current: %d", state);

			req_state = rand() % 5;
			success = hb.setState(req_state);

			ROS_INFO("setState (%u -> %u): %u", state, req_state, success);
		}

		loop_rate.sleep();
	}

	return 0;
}
