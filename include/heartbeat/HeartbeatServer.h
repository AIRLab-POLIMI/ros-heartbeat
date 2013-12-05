#ifndef HEARTBEAT_SERVER_H
#define HEARTBEAT_SERVER_H

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "heartbeat/State.h"

class HeartbeatServer {
private:
	ros::NodeHandle& nh_;
	ros::CallbackQueue callback_queue_;
	ros::AsyncSpinner spinner_;
	ros::Publisher pub_;
	ros::ServiceServer service_;
	ros::Timer timer_;
	heartbeat::State state_;
	double timeout_;

	void timer_callback(const ros::TimerEvent&);
	void heartbeat_callback(const heartbeat::State::ConstPtr& msg);
public:
	HeartbeatServer(ros::NodeHandle& nh);
	~HeartbeatServer(void);
	void start(void);
	void stop(void);
};

#endif /* HEARTBEAT_SERVER_H */
