#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "traj_gen");
    ros::NodeHandle n;
	ros::Publisher path_pub = n.advertise<std_msgs::Float64>("/two_link/joint1_position_controller/command", 100);

    ros::Rate loop_rate(1000);

	std_msgs::Float64 msg;

	while(ros::ok()){
		msg.data += 0.01;
		path_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

