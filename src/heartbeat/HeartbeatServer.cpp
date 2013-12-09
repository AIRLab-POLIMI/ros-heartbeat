#include "heartbeat/HeartbeatServer.h"
#include "ros/callback_queue.h"
#include "heartbeat/Heartbeat.h"
#include "heartbeat/State.h"
#include "heartbeat/SetState.h"

void HeartbeatServer::heartbeat_callback(
		const heartbeat::Heartbeat::ConstPtr& msg) {
	std::map<std::string, ros::Timer>::iterator it;

	it = _registered_nodes.find(msg->node_name.data);

	if (it != _registered_nodes.end()) {
		ros::Timer& timer = it->second;
		timer.stop();
		timer.start();
	}

	ROS_INFO("Heartbeat from node %s", msg->node_name.data.c_str());
}

void HeartbeatServer::heartbeat_timeout(const ros::TimerEvent&) {
	_state.value = heartbeat::State::TIMEOUT;
	_state_pub.publish(_state);
	ROS_WARN("Heartbeat timeout!");
}

bool HeartbeatServer::set_state(heartbeat::SetState::Request &req,
		heartbeat::SetState::Response &res) {

	if (req.from.value != _state.value) {
		ROS_INFO("State transition %u -> %u rejected: current state is %u",
				req.from.value, req.to.value, _state.value);
	} else if ((_state.value == heartbeat::State::UNINIT)
			&& (req.to.value != heartbeat::State::OK)) {
		ROS_INFO("State transition not allowed: %u -> %u", req.from.value,
				req.to.value);
	} else {
		_state.value = req.to.value;
		_state_pub.publish(_state);
		ROS_INFO("State updated: %u -> %u", req.from.value, _state.value);
	}

	res.current.value = _state.value;
	return true;
}

bool HeartbeatServer::register_node(heartbeat::RegisterNode::Request &req,
		heartbeat::RegisterNode::Response &res) {

	if (_registered_nodes.find(req.node_name.data) == _registered_nodes.end()) {
		ros::Timer timer = _nh.createTimer(ros::Duration(req.timeout.data),
				&HeartbeatServer::heartbeat_timeout, this);
		_registered_nodes.insert(
				std::pair<std::string, ros::Timer>(req.node_name.data, timer));
		res.success = true;
		ROS_INFO("Node registered: %s", req.node_name.data.c_str());
	} else {
		res.success = false;
		ROS_INFO("Node registered: %s", req.node_name.data.c_str());
	}

	return true;
}

bool HeartbeatServer::unregister_node(heartbeat::UnregisterNode::Request &req,
		heartbeat::UnregisterNode::Response &res) {

	if (_registered_nodes.erase(req.node_name.data)) {
		res.success = true;
	} else {
		res.success = false;
	}
	return true;
}

HeartbeatServer::HeartbeatServer(ros::NodeHandle& nh) :
		_nh(nh), _spinner(1, &_callback_queue) {
	ros::TimerOptions timer_ops;
	ros::AdvertiseOptions adv_ops;
	ros::SubscribeOptions sub_ops;

	ros::param::param<float>("/heartbeat/state_update_period", _state_period,
			1.0);

	sub_ops.init<heartbeat::Heartbeat>("heartbeat", 1,
			boost::bind(&HeartbeatServer::heartbeat_callback, this, _1));
	sub_ops.tracked_object = ros::VoidPtr();
	sub_ops.callback_queue = &_callback_queue;
	_heartbeat_sub = _nh.subscribe(sub_ops);

	_state_pub = _nh.advertise<heartbeat::State>("state", 10);

	_set_state_service = _nh.advertiseService("/heartbeat/set_state",
			&HeartbeatServer::set_state, this);
	_register_node_service = _nh.advertiseService("/heartbeat/register_node",
			&HeartbeatServer::register_node, this);
	_unregister_node_service = _nh.advertiseService(
			"/heartbeat/unregister_node", &HeartbeatServer::unregister_node,
			this);
}

HeartbeatServer::~HeartbeatServer(void) {
	stop();
}

void HeartbeatServer::start(void) {
	_spinner.start();
}

void HeartbeatServer::stop(void) {
	_spinner.stop();
}

void HeartbeatServer::spin(void) {
	_state_pub.publish(_state);
}
