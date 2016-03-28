// (TODO update this legal notice)
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

// Setup ros node handlers, message type variables and odom publisher/brodcaster
ros::NodeHandle n;
tf::TransformBroadcaster odom_broadcaster;
geometry_msgs::TransformStamped odom_trans;
ros::Publisher odom_pub;

// Reset topic_path,
std::string reset_topic, frame_id, base_link;

// Times
ros::Time current_time, last_time;

// Decide if we want to average two wheels on each side (true) or not (false)
bool average_ticks_on_side;

// The number of times per second (hz) odom will update
int frequency;

// Used to prevent callbacks from accessing variables
// before they are initialised
bool initialised = false;

// Most nav stacks will not accept an odom of under 30 hertz
int minimum_odom_refresh;

// Index numbers of left and right encoders for the case
// where two or more encoders exist on the same Phidget device,
// in the case that there is only one encoder on each side, use only 0 indexes
int encoder_index_left0;
int encoder_index_left1;
int encoder_index_right0;
int encoder_index_right1;

// These are used internally for averaging sides
int left0_count = 0;
int left1_count = 0;
int right0_count = 0;
int right1_count = 0;

// Should we announce every encoder count that arrives?
bool verbose;

// Det to true remotly during runtime to reset odom
bool reset = false;

// Normally on a differential drive system to when moving
// forwards the wheels are rotating in opposite directions
int encoder_direction_left0;
int encoder_direction_left1;
int encoder_direction_right0;
int encoder_direction_right1;

// Distance between the wheel centres
double wheelbase_mm;

// How many ticks in a rev
double ticks_per_rev;

// Wheel diameter
double wheel_diam_mm;

// Friction coefficient, necessary to skid drive
double friction_coefficient;

// Encoder counts
int current_encoder_count_left = 0;
int current_encoder_count_right = 0;
int previous_encoder_count_left = 0;
int previous_encoder_count_right = 0;

// Encoder counts per millimetre
double left_encoder_counts_per_mm = .1227;
double right_encoder_counts_per_mm =.1227;

// Subscribers
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

double rotation_offset;

/*!
 * brief callback when the left or right encoder count changes
 * param ptr encoder parameters
 */
void encoderCallback(const phidgets_motion_control::encoder_params::ConstPtr& ptr)
{
	if (initialised) {
		phidgets_motion_control::encoder_params e = *ptr;

		// Calculate the ticks on one side by either the average of one side or a
		// single encoder
		if(average_ticks_on_side) { // average encoders on one side
			if (e.index == encoder_index_left0) {
				current_encoder_count_left =(int)(((e.count * encoder_direction_left0) +
					(left1_count) )
					/ 2 );
					left0_count = e.count * encoder_direction_left0;
			} else if (e.index == encoder_index_left1) {
				current_encoder_count_left =(int)(((e.count * encoder_direction_left1) +
					(left0_count) )
					/ 2 );
					left1_count = e.count * encoder_direction_left1;
			} else if (e.index == encoder_index_right0) {
				current_encoder_count_right =(int)(((e.count * encoder_direction_right0) +
					(right0_count) )
					/ 2 );
					right0_count = e.count * encoder_direction_right0;
			} else if (e.index == encoder_index_right1) {
				current_encoder_count_right =(int)(((e.count * encoder_direction_right1) +
					(right0_count) )
					/ 2 );
					right1_count = e.count * encoder_direction_right1;
				}
		} else { // Only pay attention to two encoders
			if (e.index == encoder_index_left0 ) {
		  	current_encoder_count_left = e.count * encoder_direction_left0;
			} else if (e.index == encoder_index_right0 ) {
				current_encoder_count_right = e.count * encoder_direction_right0;
			}
		}
	}
	return;
}

/*!
 * param connects to a phidgets high speed encoder
 *        device which contains two or more encoders
 */
bool subscribe_to_encoders_by_index() {
  bool success = true;
  ros::NodeHandle n;
	ros::NodeHandle nh("~");
	std::string topic_path = "phidgets/";
	nh.param("topic_path", topic_path);
	std::string encodername = "encoder";
	nh.param("encodername", encodername);
  std::string encoder_topic_base = topic_path + encodername;
	nh.param("encoder_topic", encoder_topic_base);

	encoders_sub = n.subscribe(encoder_topic_base, 300, encoderCallback);

	return(success);
}

// Updates the global odometry values,
// passed the delta time in seconds
inline void update_odom(double dt) {

	if((current_encoder_count_left - previous_encoder_count_left) == 0 &&
		(current_encoder_count_right - previous_encoder_count_right) == 0) {

		delta_x = 0;
		delta_y = 0;
    delta_theta = 0;
		return;
	}

	double wheel_base_m = (wheelbase_mm / 1000);

	double friction_coefficient = 0;

	double wheel_diam_m = (wheel_diam_mm / 1000);

  double right_wheel_travel = (current_encoder_count_right - previous_encoder_count_right) * ((wheel_diam_m * M_PI) / ticks_per_rev);
  double left_wheel_travel = (current_encoder_count_left - previous_encoder_count_left) * ((wheel_diam_m * M_PI) / ticks_per_rev);

	if( (left_wheel_travel - right_wheel_travel) == 0){
		if(left_wheel_travel == 0 && right_wheel_travel == 0) {
			delta_x = 0;
			delta_y = 0;
			delta_theta = 0;
		} else {
			delta_x = cos(theta) * left_wheel_travel;
			delta_y = sin(theta) * right_wheel_travel;
			delta_theta = 0;
		}
		return;
	}

	if((current_encoder_count_left - previous_encoder_count_left) ==
		(current_encoder_count_right - previous_encoder_count_right)) {

		delta_x = sin(theta) * right_wheel_travel;
		delta_y = cos(theta) * right_wheel_travel;
		delta_theta = 0;
		return;
	}

	double pivot_angl = ( ( right_wheel_travel - left_wheel_travel ) / wheel_base_m )
		* (1 - friction_coefficient);
	double pivot_center = ( ( wheel_base_m / 2 ) *  ( ( left_wheel_travel + right_wheel_travel )
		/ ( left_wheel_travel - right_wheel_travel ) ) ) * (1 + friction_coefficient);

  double x_instantanous_center_of_curvature = x - ( pivot_center * cos(theta) );
	double y_instantanous_center_of_curvature = y + ( pivot_center * sin(theta) );

	// The following represent matrix and matrix transformations, done in line for
	// ease of use

	//rTp
	double rTp[3][3];
  rTp[0][0] = cos( pivot_angl * dt );
  rTp[0][1] = -1 * sin( pivot_angl * dt );
	rTp[0][2] = 0;
  rTp[1][0] = sin( pivot_angl * dt );
  rTp[1][1] = cos( pivot_angl * dt );
	rTp[1][2] = 0;
	rTp[2][0] = 0;
	rTp[2][1] = 0;
	rTp[2][2] = 1;

	//pTicc
	double pTicc[3][1];
	pTicc[0][0] = x - ( x_instantanous_center_of_curvature );
	pTicc[1][0] = y - ( y_instantanous_center_of_curvature );
	pTicc[2][0] = theta;

	//iccTn
	double iccTn[3][1];
	iccTn[0][0] = x_instantanous_center_of_curvature;
	iccTn[1][0] = y_instantanous_center_of_curvature;
	iccTn[2][0] = dt * pivot_angl;

	//rTn = rtp * pticc + iccTn
	double rTn[3][1];
	rTn[0][0] = rTp[0][0] * pTicc[0][0] + rTp[0][1] * pTicc[0][0] + iccTn[0][0];
	rTn[1][0] = rTp[1][0] * pTicc[1][0] + rTp[1][1] * pTicc[1][0] + iccTn[1][0];
	rTn[2][0] = pTicc[2][0] + iccTn[2][0];

	// Calculate delta based off of above matrix transform
  delta_x = (rTn[0][0] - x);
  delta_y = (rTn[1][0] - y);
	delta_theta = rTn[2][0] - theta;

	previous_encoder_count_left = current_encoder_count_left;
	previous_encoder_count_right = current_encoder_count_right;

	return;
}

void odomRefreshCallback(const ros::TimerEvent& event) {

	// Handle and update time
	current_time = ros::Time::now();
	double dt = ((double)current_time.toSec() - (double)last_time.toSec());

	// reset the pose
	n.param(reset_topic, reset, false);

	if (reset) {
		x = 0;
		y = 0;
		theta = 0;
		delta_x = 0;
		delta_y = 0;
		delta_theta = 0;
		n.setParam(reset_topic, false);
	}

	// Update the velocity estimate based upon
	update_odom(dt);

	// Compute odometry in a typical way given
	// the velocities of the robot
	x += delta_x;
	y += delta_y;
	theta += -delta_theta;

	// Get Quaternion from rpy
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

	// Publish the transform over tf
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
	odom.twist.twist.linear.x = delta_x / dt;
	odom.twist.twist.linear.y = delta_y / dt;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = delta_theta / dt;

	// publish odom
	odom_pub.publish(odom);

	last_time = current_time;
}

int main(int argc, char** argv)
{
	ROS_INFO("ROS Odom node begining...");

	ros::init(argc, argv, "phidgets_odometry");

	// Setup NodeHandle and related topic params
	ros::NodeHandle nh("~");
	std::string name;
	nh.param<std::string>("name", name, "odom");
	nh.param<std::string>("reset_topic", reset_topic, "odometry/reset");
	nh.param("rotation", rotation_offset, 0.0);

	// Set reset topic to false, set to true to reset odom during runtime
	n.setParam(reset_topic, false);

	odom_pub =
		n.advertise<nav_msgs::Odometry>(name, 50);
	tf::TransformBroadcaster odom_broadcaster;

	// Get encoder indexes
	// First see if we need to get 2 or 4 encoder indexes to average sides
	nh.param("average_ticks_on_side", average_ticks_on_side, false);

	if(average_ticks_on_side) {
		nh.param("encoder_index_left0", encoder_index_left0, 1);
		nh.param("encoder_index_left1", encoder_index_left1, 2);

		nh.param("encoder_index_right0", encoder_index_right0, 3);
		nh.param("encoder_index_right1", encoder_index_right1, 4);

		nh.param("encoder_direction_left0", encoder_direction_left0, 1);
		nh.param("encoder_direction_right0", encoder_direction_right0, -1);

		nh.param("encoder_direction_left1", encoder_direction_left1, 1);
		nh.param("encoder_direction_right1", encoder_direction_right1, -1);

	} else {
		nh.param("encoder_index_left0", encoder_index_left0, 0);
		nh.param("encoder_index_right0", encoder_index_right0, 1);

		nh.param("encoder_direction_left0", encoder_direction_left0, 1);
		nh.param("encoder_direction_right0", encoder_direction_right0, -1);
	}

	// Get neccessary distance info for calculating odom
	nh.param("ticks_per_rev", ticks_per_rev, 100.0);
	nh.param("wheel_diam_mm", wheel_diam_mm, 1.0);

	// Connect to the encoders
	std::string topic_path;
	std::string encodername;
	nh.param<std::string>("topic_path", topic_path, "phidgets/");
	nh.param<std::string>("encoder_name", encodername, "encoder");
  std::string encoder_topic_base = topic_path + encodername;
	nh.param<std::string>("encoder_topic", encoder_topic_base, encoder_topic_base);

	encoders_sub = n.subscribe(encoder_topic_base, 300, encoderCallback);

	// Setup nav and odom topics
  nh.param<std::string>("base_link", base_link, "base_link");
  nh.param<std::string>("frame_id", frame_id, "odom");
	nh.param("counts_per_mm_left", left_encoder_counts_per_mm, 1.0);
	nh.param("counts_per_mm_right", right_encoder_counts_per_mm, 1.0);
	nh.param("wheelbase", wheelbase_mm, 100.0);
	nh.param("friction_coefficient", friction_coefficient, .1);

	// Get verbosity level
  nh.param("verbose", verbose, false);

	// Get update rate for odom
  nh.param("frequency", frequency, 30);
	nh.param("minimum_odom_refresh", minimum_odom_refresh, 30);

	// Most nav stacks will not accept an odom of under 30 hertz
	if( frequency < minimum_odom_refresh ) {
		ROS_WARN("Odom update frequency below %i ( > %i ), setting to %i",
			minimum_odom_refresh, frequency, minimum_odom_refresh );
		frequency = minimum_odom_refresh;
	}

	// Set frame id and base link path strings
	odom_trans.header.frame_id = frame_id;
	odom_trans.child_frame_id = base_link;

	// Times to calculate delta seconds for odom
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	//Tell the encoder callback that we are ready to start counting ticks
	initialised = true;

  // Kick off the timer that will perodicaly call odom to update and push transforms
	ros::Timer odom_timer = nh.createTimer(ros::Duration(frequency), odomRefreshCallback);
	ros::spin();

	return 0;
}
