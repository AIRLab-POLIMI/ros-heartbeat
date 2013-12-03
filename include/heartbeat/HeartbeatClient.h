#ifndef HEARTBEAT_CLIENT_H
#define HEARTBEAT_CLIENT_H

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "heartbeat/State.h"

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

inline
heartbeat::State HeartbeatClient::getState(void) {
	return state_;
}

#endif /* HEARTBEAT_CLIENT_H */
