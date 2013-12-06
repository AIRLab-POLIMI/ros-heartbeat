#include "heartbeat/HeartbeatClient.h"
#include "ros/callback_queue.h"
#include "heartbeat/SetState.h"


void HeartbeatClient::timer_callback(const ros::TimerEvent&) {
	ROS_INFO("Timeout!");
}

void HeartbeatClient::state_callback(const heartbeat::State::ConstPtr& msg) {
	_state_timer.stop();
	_state_timer.start();
	_state.value = msg->value;
	ROS_INFO("Received: %u", msg->value);
}

HeartbeatClient::HeartbeatClient(ros::NodeHandle& nh, float heartbeat_timeout) :
		_nh(nh), _heartbeat_timeout(heartbeat_timeout), _spinner(1, &_callback_queue) {
	ros::TimerOptions timer_ops;
	ros::SubscribeOptions sub_ops;

	ros::param::param<float>("/heartbeat/timeout", _state_timeout, 1.0);
	ros::param::param<std::string>("/heartbeat/heartbeat_topic", _heartbeat_topic, "heartbeat");
	ros::param::param<std::string>("/heartbeat/state_topic", _state_topic, "state");

	timer_ops.autostart = false;
	timer_ops.callback = boost::bind(&HeartbeatClient::timer_callback, this, _1);
	timer_ops.callback_queue = &_callback_queue;
	timer_ops.oneshot = false;
	timer_ops.period = ros::Duration(_state_timeout);
	timer_ops.tracked_object = ros::VoidPtr();
	_state_timer = _nh.createTimer(timer_ops);

	sub_ops.init<heartbeat::State>(_state_topic, 1, boost::bind(&HeartbeatClient::state_callback, this, _1));
	sub_ops.tracked_object = ros::VoidPtr();
	sub_ops.callback_queue = &_callback_queue;
	_state_sub = _nh.subscribe(sub_ops);

	_state_service = _nh.serviceClient<heartbeat::SetState>("set_state");

	if (_heartbeat_timeout != 0) {

	}
}

HeartbeatClient::~HeartbeatClient(void) {
	stop();
}


void HeartbeatClient::start(void) {
	_spinner.start();
	_state_timer.start();
}

void HeartbeatClient::stop(void) {
	_state_timer.stop();
	_spinner.stop();
}

bool HeartbeatClient::setState(heartbeat::State& to) {
	heartbeat::SetState req_state;

	req_state.request.from.value = _state.value;
	req_state.request.to.value = to.value;

	if (!_state_service.call(req_state)) {
		return false;
	}

	if (req_state.response.current.value != to.value) {
		_state.value = req_state.response.current.value;
		return false;
	}

	return true;
}

void HeartbeatClient::alive(void) {

}
