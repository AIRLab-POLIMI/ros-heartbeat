#include "ros/ros.h"
#include "heartbeat/State.h"
#include "heartbeat/SetState.h"

#include <sstream>

heartbeat::State state;
ros::Publisher heartbeat_pub;

bool set_state(heartbeat::SetState::Request &req,
		heartbeat::SetState::Response &res) {
	
	if ((state.value == heartbeat::State::UNINIT) && (req.state.value != heartbeat::State::OK)) {
		res.success.data = false;
	} else {
		state = req.state;
		res.success.data = true;
		heartbeat_pub.publish(state);
	}
	
	ROS_INFO("set_state request: %u -> %u", state.value, req.state.value);
	ROS_INFO("sending back response: [%u]", res.success.data);
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
