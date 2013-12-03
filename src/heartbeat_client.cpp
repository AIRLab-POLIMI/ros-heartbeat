#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "heartbeat/State.h"
#include "heartbeat/SetState.h"

#include <stdlib.h>

class HeartbeatClient {
private:
	ros::NodeHandle& nh_;
	ros::CallbackQueue callback_queue_;
	ros::AsyncSpinner spinner_;
	ros::Subscriber sub_;
	ros::ServiceClient service_;
	ros::Timer timer_;
	heartbeat::State state_;
	double timeout_;

	void timer_callback(const ros::TimerEvent&);
	void heartbeat_callback(const heartbeat::State::ConstPtr& msg);
public:
	HeartbeatClient(ros::NodeHandle& nh);
	~HeartbeatClient(void);
	void start(void);
	void stop(void);
	bool setState(heartbeat::State& state);
	heartbeat::State getState(void);
};

void HeartbeatClient::timer_callback(const ros::TimerEvent&) {
	ROS_INFO("Timeout!");
}

void HeartbeatClient::heartbeat_callback(const heartbeat::State::ConstPtr& msg) {
	timer_.stop();
	timer_.start();
	state_.value = msg->value;
	ROS_INFO("Received: %u", msg->value);
}

HeartbeatClient::HeartbeatClient(ros::NodeHandle& nh) :
		nh_(nh), spinner_(1, &callback_queue_) {
	ros::TimerOptions timer_ops;
	ros::SubscribeOptions sub_ops;

	ros::param::param<double>("/heartbeat/timeout", timeout_, 1.0);

	timer_ops.autostart = false;
	timer_ops.callback = boost::bind(&HeartbeatClient::timer_callback, this, _1);
	timer_ops.callback_queue = &callback_queue_;
	timer_ops.oneshot = false;
	timer_ops.period = ros::Duration(timeout_);
	timer_ops.tracked_object = ros::VoidPtr();
	timer_ = nh_.createTimer(timer_ops);

	sub_ops.init<heartbeat::State>("heartbeat", 1, boost::bind(&HeartbeatClient::heartbeat_callback, this, _1));
	sub_ops.tracked_object = ros::VoidPtr();
	sub_ops.callback_queue = &callback_queue_;
	sub_ = nh_.subscribe(sub_ops);

	service_ = nh_.serviceClient<heartbeat::SetState>("set_state");
}

HeartbeatClient::~HeartbeatClient(void) {
	stop();
}


void HeartbeatClient::start(void) {
	spinner_.start();
	timer_.start();
}

void HeartbeatClient::stop(void) {
	timer_.stop();
	spinner_.stop();
}

bool HeartbeatClient::setState(heartbeat::State& state) {
	heartbeat::SetState req_state;

	req_state.request.state.value = state.value;

	if (!service_.call(req_state)) {
		return false;
	}

	return req_state.response.success.data;
}

heartbeat::State HeartbeatClient::getState(void) {
	return state_;
}

/*
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
 */


void timerCallback(const ros::TimerEvent& e) {
	ROS_INFO("Timeout!");
}

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
