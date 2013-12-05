#include "ros/ros.h"
#include "heartbeat/State.h"
#include "heartbeat/SetState.h"

#include <sstream>

heartbeat::State state;
ros::Publisher heartbeat_pub;

bool set_state(heartbeat::SetState::Request &req,
		heartbeat::SetState::Response &res) {

	if (req.from.value != state.value) {
		ROS_INFO("State transition %u -> %u rejected: current state is %u", req.from.value, req.to.value, state.value);
	} else if ((state.value == heartbeat::State::UNINIT) && (req.to.value != heartbeat::State::OK)) {
		ROS_INFO("State transition not allowed: %u -> %u", req.from.value, req.to.value);
	} else {
		state.value = req.to.value;
		heartbeat_pub.publish(state);
		ROS_INFO("State updated: %u -> %u", req.from.value, state.value);
	}
	
	res.current.value = state.value;
	return true;
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "heartbeat_server");

	state.value = heartbeat::State::UNINIT;

	ros::NodeHandle n;
	ros::Rate loop_rate(2);
	heartbeat_pub = n.advertise<heartbeat::State>("heartbeat",
			10);
	ros::ServiceServer service = n.advertiseService("set_state", set_state);

	while (ros::ok()) {
		ROS_INFO("%d", state.value);

		heartbeat_pub.publish(state);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
