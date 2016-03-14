/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Phidgets based wheel odometry for a differential drive system
 *  For use with a pair of Phidgets high speed encoders
 *  See http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Odom
 *  Copyright (c) 2010, Bob Mottram
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "phidgets_motion_control/encoder_params.h"

// used to prevent callbacks from accessing variables
// before they are initialised
bool initialised = false;

// index numbers of left and right encoders for the case
// where two or more encoders exist on the same Phidget device
int encoder_index_left=-1;
int encoder_index_right=-1;

// should we announce every encoder count that arrives?
bool verbose = true;

// normally on a differential drive system to when moving
//  forwards the wheels are rotating in opposite directions
int encoder_direction_left = -1;
int encoder_direction_right = 1;

// distance between the wheel centres
double wheelbase_mm = 400;

// encoder counts
int current_encoder_count_left = 0;
int current_encoder_count_right = 0;
int previous_encoder_count_left = 0;
int previous_encoder_count_right = 0;

// keep track if the initial encoder values so that relative
// movement can be reported
int start_encoder_count_left = 0;
int start_encoder_count_right = 0;

// encoder counts per millimetre
double left_encoder_counts_per_mm = 0;
double right_encoder_counts_per_mm = 0;

ros::Subscriber left_encoder_sub;
ros::Subscriber right_encoder_sub;
ros::Subscriber encoders_sub;

// pose estimate
double x = 0.0;
double y = 0.0;
double theta = 0.0;

// velocity estimate
double delta_x = 0.1;
double delta_y = -0.1;
double delta_theta = 0.1;

double rotation_offset=0;

// Update the left encoder count
void update_encoder_left(int count)
{
	current_encoder_count_left = count * encoder_direction_left;
	if (start_encoder_count_left == 0) {
		start_encoder_count_left = current_encoder_count_left;
	}
	if (verbose) {
		ROS_INFO("Left Encoder Count %d",
				current_encoder_count_left -
				start_encoder_count_left);
	}
}

// Update the right encoder count
void update_encoder_right(int count)
{
	current_encoder_count_right =
		count * encoder_direction_right;
	if (start_encoder_count_right == 0) {
		start_encoder_count_right = current_encoder_count_right;
	}
	if (verbose) {
		ROS_INFO("Right Encoder Count %d",
				current_encoder_count_right -
				start_encoder_count_right);
	}
}

/*!
 * \brief callback when the left or right encoder count changes
 * \param ptr encoder parameters
 */

void encoderCallback(const phidgets_motion_control::encoder_params::ConstPtr& ptr)
{

	if (initialised) {
		phidgets_motion_control::encoder_params e = *ptr;
		if (e.index == encoder_index_left) {
			update_encoder_left(e.count);
		} else if (e.index == encoder_index_right) {
			update_encoder_right(e.count);
		}
	}
}

/*!
 * \brief callback when the left encoder count changes
 * \param ptr encoder parameters
 */
void leftEncoderCallback(const phidgets_motion_control::encoder_params::ConstPtr& ptr)
{
	if (initialised) {
		phidgets_motion_control::encoder_params e = *ptr;
		update_encoder_left(e.count);
	}
}

/*!
 * \brief callback when the right encoder count changes
 * \param ptr encoder parameters
 */
void rightEncoderCallback(const phidgets_motion_control::encoder_params::ConstPtr& ptr)
{
	if (initialised) {
		phidgets_motion_control::encoder_params e = *ptr;
		update_encoder_right(e.count);
	}
}

/*!
 * \param connects to a phidgets high speed encoder
 *        device which contains two or more encoders
 */

bool subscribe_to_encoders_by_index()
   {
   	bool success = true;
   	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	std::string topic_path = "phidgets/";
	nh.getParam("topic_path", topic_path);
	std::string encodername = "encoder";
	nh.getParam("encodername", encodername);
  	std::string encoder_topic_base = topic_path + encodername;
	nh.getParam("encoder_topic", encoder_topic_base);

	encoders_sub = n.subscribe(encoder_topic_base, 300, encoderCallback);

	return(success);
}



double distance_difference_traveled_x = 0;
double distance_difference_traveled_y = 0;
double pivot_radius = 0;

//TODO the primatives in this function are overkill
void update_velocities(double delta_seconds) {
	ROS_DEBUG("curr_encoder_count_left %i, right %i", current_encoder_count_left, current_encoder_count_right);

	//const double distance_per_((WHEEL DIAM) * math.PI)/3200(TODO overall ticks per rev);		
	//TODO parameterize the below coefficent
	double friction_coefficient = .1;

	double left_encoder_counts_per_m = (left_encoder_counts_per_mm / 1000);
	double right_encoder_counts_per_m = (right_encoder_counts_per_mm / 1000);

	double wheelbase_m = (wheelbase_mm / 1000);
	//TODO subtract start ticks
	double delta_ticks_left = current_encoder_count_left - previous_encoder_count_left;
	double delta_ticks_right = current_encoder_count_right - previous_encoder_count_right;
	
	//TODO remove this debug
	


	ROS_DEBUG("delta ticks l/r: %i / %i", delta_ticks_left, delta_ticks_right);
	

	double distance_left = delta_ticks_left * left_encoder_counts_per_m;
	double distance_right = delta_ticks_right * right_encoder_counts_per_m;

	double pivot_angle = (distance_left - distance_right)/wheelbase_m;

	double distance_left_with_friction = distance_left + distance_left * pivot_angle * friction_coefficient;
	double distance_right_with_friction = distance_right - distance_right * pivot_angle * friction_coefficient;
	
	double angle_with_friction = (distance_left_with_friction - distance_right_with_friction) / wheelbase_m ;

	double distance_traveled_x;
	double distance_traveled_y;

	if(distance_left_with_friction == 0 && distance_right_with_friction == 0 ) {
		distance_traveled_x = 0;
		distance_traveled_y = 0;
	}else if (distance_left_with_friction - distance_right_with_friction == 0) {
		distance_traveled_x = distance_left_with_friction;
		distance_traveled_y = 0;
	} else {
		double pivot_radius = (wheelbase_m / 2) * ((distance_left_with_friction + distance_right_with_friction) / (distance_left_with_friction - distance_right_with_friction));
		distance_traveled_x = (pivot_radius * cos(pivot_angle)) * sqrt(pow(distance_left_with_friction, 2) + pow(distance_right_with_friction, 2));
		distance_traveled_y = (pivot_radius * sin(pivot_angle)) * sqrt(pow(distance_left_with_friction, 2) + pow(distance_right_with_friction, 2));
	}

	distance_difference_traveled_x = distance_traveled_x * delta_seconds;
	distance_difference_traveled_y = distance_traveled_y * delta_seconds;

	previous_encoder_count_left = current_encoder_count_left;
	previous_encoder_count_right = current_encoder_count_right;

}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "phidgets_odometry");

	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	std::string name = "odom";
	nh.getParam("name", name);

	std::string reset_topic = "odometry/reset";
	nh.getParam("reset_topic", reset_topic);

	nh.getParam("rotation", rotation_offset);

	n.setParam(reset_topic, false);

	// TODO paramaterize num to keep in buffer
	ros::Publisher odom_pub =
		n.advertise<nav_msgs::Odometry>(name, 50);
	tf::TransformBroadcaster odom_broadcaster;

	// get encoder indexes
	nh.getParam("encoderindexleft", encoder_index_left);
	nh.getParam("encoderindexright", encoder_index_right);

	nh.getParam("encoderdirectionleft", encoder_direction_left);
	nh.getParam("encoderdirectionright", encoder_direction_right);


	// connect to the encoders
	std::string topic_path = "phidgets/";
	nh.getParam("topic_path", topic_path);
	std::string encodername = "encoder";
	nh.getParam("encodername", encodername);
  	std::string encoder_topic_base = topic_path + encodername;
	nh.getParam("encoder_topic", encoder_topic_base);

	encoders_sub = n.subscribe(encoder_topic_base, 300, encoderCallback);


	// Setup nav and odom stuff
	std::string base_link = "base_link";
	nh.getParam("base_link", base_link);
	std::string frame_id = "odom";
	nh.getParam("frame_id", frame_id);
	nh.getParam("countspermmleft",
			left_encoder_counts_per_mm);
	nh.getParam("countspermmright",
			right_encoder_counts_per_mm);

	nh.getParam("wheelbase", wheelbase_mm);

	// Get verbosity level
	nh.getParam("verbose", verbose);

	int frequency = 20;
	nh.getParam("frequency", frequency);

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = frame_id;
	odom_trans.child_frame_id = base_link;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	initialised = true;

	ros::Rate update_rate(frequency);
	while(ros::ok()){


		
		// Handle and update time
		current_time = ros::Time::now();
		//TODO remove test section below
		double dt = (current_time - last_time).toSec();
		//double delta_x = ( 0.4 * cos(theta) - 0.0 * sin(theta) ) * dt;
		//double delta_y = ( .4 * sin(theta) + 0.0 * cos(theta) ) * dt;
		//double delta_th = 0.4 * dt;

		// reset the pose
		bool reset = false;
		n.getParam(reset_topic, reset);

		if (reset) {
			x = 0;
			y = 0;
			theta = 0;
			delta_x = 0;
			delta_y = 0;
			delta_theta = 0;
			n.setParam(reset_topic, false);
			start_encoder_count_left = 0;
			start_encoder_count_right = 0;
		}

		// update the velocity estimate based upon
		// TODO encoder values
		update_velocities(dt);

		// compute odometry in a typical way given
		// the velocities of the robot
		x += distance_difference_traveled_x;
		y += distance_difference_traveled_y;
		theta += pivot_radius;

		// get Quaternion from rpy
		geometry_msgs::Quaternion odom_quat =
			tf::createQuaternionMsgFromYaw(theta);

		// first, we'll publish the transform over tf
		odom_trans.header.stamp = current_time;

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		// send the transform
		odom_broadcaster.sendTransform(odom_trans);

		// publish the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = frame_id;

		// set position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		// set the velocity
		odom.child_frame_id = base_link;
		odom.twist.twist.linear.x = distance_difference_traveled_x / dt;
		odom.twist.twist.linear.y = distance_difference_traveled_y / dt;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;	
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = pivot_radius / dt;

		// publish odom
		odom_pub.publish(odom);

		last_time = current_time;
		ros::spinOnce();
		update_rate.sleep();
	}
	return 0;
}


