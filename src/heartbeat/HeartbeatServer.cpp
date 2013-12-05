#include "heartbeat/HeartbeatServer.h"
#include "ros/callback_queue.h"
#include "heartbeat/SetState.h"


void HeartbeatServer::timer_callback(const ros::TimerEvent&) {
	ROS_INFO("Timeout!");
}

void HeartbeatServer::heartbeat_callback(const heartbeat::State::ConstPtr& msg) {
	timer_.stop();
	timer_.start();
	state_.value = msg->value;
	ROS_INFO("Received: %u", msg->value);
}

HeartbeatServer::HeartbeatServer(ros::NodeHandle& nh) :
		nh_(nh), spinner_(1, &callback_queue_) {
	ros::TimerOptions timer_ops;
	ros::SubscribeOptions sub_ops;

	ros::param::param<double>("/heartbeat/timeout", timeout_, 1.0);

	timer_ops.autostart = false;
	timer_ops.callback = boost::bind(&HeartbeatServer::timer_callback, this, _1);
	timer_ops.callback_queue = &callback_queue_;
	timer_ops.oneshot = false;
	timer_ops.period = ros::Duration(timeout_);
	timer_ops.tracked_object = ros::VoidPtr();
	timer_ = nh_.createTimer(timer_ops);

	sub_ops.init<heartbeat::State>("heartbeat", 1, boost::bind(&HeartbeatServer::heartbeat_callback, this, _1));
	sub_ops.tracked_object = ros::VoidPtr();
	sub_ops.callback_queue = &callback_queue_;
	sub_ = nh_.subscribe(sub_ops);

	service_ = nh_.serviceClient<heartbeat::SetState>("set_state");
}

HeartbeatServer::~HeartbeatServer(void) {
	stop();
}


void HeartbeatServer::start(void) {
	spinner_.start();
	timer_.start();
}

void HeartbeatServer::stop(void) {
	timer_.stop();
	spinner_.stop();
}

bool HeartbeatServer::setState(heartbeat::State& state) {
	heartbeat::SetState req_state;

	req_state.request.state.value = state.value;

	if (!service_.call(req_state)) {
		return false;
	}

	return req_state.response.success.data;
}
