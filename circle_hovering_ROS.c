#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
struct pose {
	float x;
	float y;
	float z;
	float yaw;
};
float c_position[] = { 2,2,1 };
Vector3d w(0, 0, 1);
//Vector3d v(0,0,0);
Vector3d position;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pos;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	current_pos = *msg;
}

float dist(geometry_msgs::PoseStamped des, mavros_msgs::PositionTarget curr) {
	float res = pow((des.pose.position.x - curr.position.x), 2) + pow((des.pose.position.y - curr.position.y), 2) + pow((des.pose.position.z - curr.position.z), 2);
	return res;
}

int main(int argc, char** argv)
{
	float result = 0;
	// ros node setting
	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh;
	ros::Rate rate(20.0);

	// regist topic and service
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
		("mavros/state", 10, state_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("mavros/local_position/pose", 10, pos_cb);

	// TODO: regist publisher
	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
		("mavros/setpoint_raw/local", 10);


	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
		("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode");

	// wait for FCU connection
	while (ros::ok() && !current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}
	int i = 0;
	// TODO: read trajectory file and save to poses
	pose poses[20];
	/*char tempString[100];
	int i=0;
	ifstream input("trajectory.txt");
	while(!input.eof()){
		input.getline(tempString,100);
		char *verify =strtok(tempString," ");
		char *lines;
		while(verify){
			lines=verify;
			verify=strtok(NULL," ");
			poses[i].x=(float)atoi(lines);
			lines=verify;
			verify=strtok(NULL," ");
			poses[i].y=(float)atoi(lines);
			lines=verify;
			verify=strtok(NULL," ");
			poses[i].z=(float)atoi(lines);
			lines=verify;
			verify=strtok(NULL," ");
			poses[i].yaw=(float)atoi(lines);
		}
		i++;
	}
*/
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	// TODO: set start des position
	mavros_msgs::PositionTarget pose;
	pose.position.x = c_position[0];
	pose.position.y = c_position[1];
	pose.position.z = c_position[2];
	pose.yaw = 3;
	i = 1;
	int flag = 0;
	while (ros::ok()) {

		if (current_state.mode != "OFFBOARD" &&
			(ros::Time::now() - last_request > ros::Duration(5.0))) {
			if (set_mode_client.call(offb_set_mode) &&
				offb_set_mode.response.mode_sent) {
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		}
		else {
			if (!current_state.armed &&
				(ros::Time::now() - last_request > ros::Duration(5.0))) {
				if (arming_client.call(arm_cmd) &&
					arm_cmd.response.success) {
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}


		if (flag == 0) {
			result = dist(current_pos, pose);
			if (result < 0.5) {
				flag = 1;
			}
		}
		else if (flag == 1) {
			result = dist(current_pos, pose);
			if (result < 0.5) {
				Vector3d t(pose.position.x, pose.position.y, pose.position.z);
				position = t;
				t = w.cross(position);
				pose.position.x = t[0];
				pose.position.y = t[1];
				pose.position.z = 1;
			}

		}
		//result=dist(current_pos,pose);
		//if(result<0.05)
	//result=dist(current_pos,pose);
		//if(result<0.05){
			 //pose.position.x = poses[i].x;
			// pose.position.y = poses[i].y;
		   //  pose.position.z = poses[i].z;
		  //   pose.yaw = poses[i].yaw;
		 //    i++;
		//}
		//if(i==6){
		//    i=0;
		//}

		// TODO: modify des position when the distance between current position and des posotion is smaller than 0.05
		// publish des_pos topic
		local_pos_pub.publish(pose);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}