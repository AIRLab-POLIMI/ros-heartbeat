#include "ros/ros.h"
#include "heartbeat/State.h"
#include "heartbeat/SetState.h"

#include <stdlib.h>

/*
class HeartbeatClient {
private:
	heartbeat::State state;
	ros::Timer timer;
	
	void timer_callback(const ros::TimerEvent&);
	void heartbeat_callback(const heartbeat::State& msg);
public:
	HeartbeatClient(ros::NodeHandle& n);
	~HeartbeatClient(void);
	heartbeat::State getState(void);
}

HeartBeat::HeartBeat(ros::NodeHandle& n) {
	timer = n.createTimer(ros::Duration(0.2), timer_callback);
	ros::Subscriber heartbeat_sub = n.subscribe<heartbeat::State>("heartbeat",10, heartbeat_callback);
}
*/

heartbeat::State state;
ros::Timer timer;

void timer_callback(const ros::TimerEvent&)
{
  ROS_INFO("Timeout!");
}

void heartbeat_callback(const heartbeat::State::ConstPtr& msg)
{
  timer.stop();
  timer.start();
  state.value = msg->value;
  ROS_INFO("Received: %u", msg->value);
}

int main(int argc, char **argv) {
	heartbeat::SetState set_state;

	ros::init(argc, argv, "heartbeat_client");

	ros::NodeHandle n;
	ros::Rate loop_rate(0.5);
	timer = n.createTimer(ros::Duration(0.6), timer_callback);
	ros::Subscriber heartbeat_sub = n.subscribe<heartbeat::State>("heartbeat",10, heartbeat_callback);
	ros::ServiceClient client = n.serviceClient<heartbeat::SetState>("set_state");

	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	while (ros::ok()) {
		ROS_INFO("Current: %d", state.value);

		set_state.request.state.value = rand() % 4;
		client.call(set_state);

		ROS_INFO("SetState (%u -> %u): %u", state.value, set_state.request.state.value, set_state.response.success.data);
		
//		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
