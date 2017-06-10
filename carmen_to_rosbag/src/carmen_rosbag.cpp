#include <fstream>
#include <string>
#include <sstream>
#include <iterator>
#include <unordered_map>
#include <cstddef>
#include <cmath>
#include <vector>
#include <initializer_list>

#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>

#include "carmen_to_rosbag/carmen_rosbag.h"

namespace 
{
	// this function maps string to integers so that they can be used in a 'switch' statement

	void initialize_for_switch( std::unordered_map<std::string, int>& string_to_number_for_switch )
	{
		string_to_number_for_switch["PARAM"] = 1;
		string_to_number_for_switch["ODOM"] = 2;
		string_to_number_for_switch["RAWLASER1"] = 3;
		string_to_number_for_switch["RAWLASER2"] = 3;
		string_to_number_for_switch["RAWLASER3"] = 3;
		string_to_number_for_switch["RAWLASER4"] = 3;
		string_to_number_for_switch["ROBOTLASER1"] = 4;
		string_to_number_for_switch["ROBOTLASER2"] = 4;
		string_to_number_for_switch["TRUEPOS"] = 5;
		string_to_number_for_switch["NMEAGGA"] = 6; 
		string_to_number_for_switch["NMEARMC"] = 7;
	}
	
	void split_string_by_space( const std::string& to_split, std::vector<std::string>& words )
	{
		std::istringstream iss(to_split);
		words = std::vector<std::string>({std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}});		
	}

	void separate_with_string( std::string& result, std::initializer_list<std::string> words, const std::string& delimiter = " " )
	{
		for (auto word : words)
			result += (word + delimiter);
	}

	template <class Iterator>
	void separate_with_string( std::string& result, Iterator begin, Iterator end, const std::string& delimiter = " " )
	{
		for (Iterator it = begin; it != end; ++it)
			result += (*it + delimiter);
	}
}

void carmen_log_to_rosbag::convert( const char* carmen_log_file_name, const char* output_bag_file_name )
{
	bag.open(output_bag_file_name, rosbag::bagmode::Write);

	std::ifstream in(carmen_log_file_name);

	// this will be needed for the switch statement; part of initialization process
	std::unordered_map<std::string, int> string_to_number_for_switch;
	initialize_for_switch(string_to_number_for_switch);

	// this is going to be a part of the name of every topic
	ros::NodeHandle nh("~");
	node_private_name = nh.resolveName("") + "/";

	seq = 0;


	// parse log file line by line
	std::string to_parse;	
	while (getline(in, to_parse))
	{
		// skip comments and empty lines
		if (to_parse[0] == '#' || to_parse == "")
			continue;		


		std::size_t pos = to_parse.find(' ');
		std::string message_type = to_parse.substr(0, pos);
		int for_switch = string_to_number_for_switch[message_type];
		switch (for_switch)
		{
			case 1:
				param(to_parse);
				continue;
			case 2:
				publish_odometry_message(to_parse);
				continue;
			case 3:
				publish_rawlaser_message(to_parse);	
				continue;
			case 4:
				publish_robotlaser_message(to_parse);	
				continue;
			case 5:
				publish_truepos_message(to_parse);
				continue;
			case 6:
				publish_NMEA_message(to_parse, "NMEAGGA_topic");	
				continue;
			case 7:
				publish_NMEA_message(to_parse, "NMEARMC_topic");
				continue;
			default:
				continue;
		}


	}

	bag.close();
}

// CARMEN format:
// ODOM x y theta tv rv accel

void carmen_log_to_rosbag::publish_odometry_message( const std::string& to_parse, const std::string& add_to_topic )
{
	// split the string

	std::vector<std::string> words;
	split_string_by_space(to_parse, words);

	ros::Time timestamp(std::stof(words[words.size() - 3]));

	// parse input data

	float x = std::stof(words[1]);
	float y = std::stof(words[2]);
	float z = 0.0;

	geometry_msgs::Vector3 vec;
	vec.x = x;
	vec.y = y;
	vec.z = z;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, std::stof(words[3]));
	
	geometry_msgs::Transform transform;
	transform.translation = vec;
	transform.rotation = odom_quat;

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.transform = transform;	
	odom_trans.header.stamp = timestamp;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.header.seq = seq++;

	tf::tfMessage tfmessage;
	tfmessage.transforms.push_back(odom_trans);
	geometry_msgs::TransformStamped& tf_m_ref = tfmessage.transforms.back();

	tf_m_ref.header.frame_id = tf::resolve("", tf_m_ref.header.frame_id);
	tf_m_ref.child_frame_id = tf::resolve("", tf_m_ref.child_frame_id);

	nav_msgs::Odometry odometry_message;
	odometry_message.header.stamp = timestamp;
	odometry_message.header.frame_id = "odom";
	odometry_message.child_frame_id = "base_link";
	odometry_message.header.seq = seq++;

	odometry_message.pose.pose.position.x = x;
	odometry_message.pose.pose.position.y = y;
	odometry_message.pose.pose.position.z = z;
	odometry_message.pose.pose.orientation = odom_quat;

	bag.write(node_private_name + "tf", timestamp, tfmessage);
	bag.write(node_private_name + std::string("ODOM") + (add_to_topic == "" ? "" : "_") + add_to_topic + "_topic", timestamp, odometry_message);

}


// CARMEN format:
// RAWLASER1 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values]
void carmen_log_to_rosbag::publish_rawlaser_message( const std::string& to_parse )
{

    // split the string
	std::vector<std::string> words;
	split_string_by_space(to_parse, words);

	sensor_msgs::LaserScan laserscan_message;
	ros::Time timestamp(std::stof(words[words.size() - 3]));
	laserscan_message.header.stamp = timestamp;
	laserscan_message.header.frame_id = "laser";
	laserscan_message.header.seq = seq++;

	laserscan_message.angle_increment = std::stof(words[4]);
	laserscan_message.angle_min = std::stof(words[2]);
	laserscan_message.angle_max = std::stof(words[2]) + std::stof(words[3]);

	laserscan_message.range_min = 0;
	laserscan_message.range_max = std::stof(words[5]);

	int num_readings = std::stoi(words[8]);
	for (int i = 0; i < num_readings; ++i)
		laserscan_message.ranges.push_back(std::stof(words[9 + i]));

	bag.write(node_private_name + "RAWLASER_topic", timestamp, laserscan_message);

}

// CARMEN format:
// TRUEPOS true_x true_y true_theta odom_x odom_y odom_theta
void carmen_log_to_rosbag::publish_truepos_message( const std::string& to_parse )
{
    // split the string
	std::vector<std::string> words;
	split_string_by_space(to_parse, words);

	ros::Time timestamp(std::stof(words[words.size() - 3]));

	geometry_msgs::PoseStamped true_pos;
	true_pos.header.seq = seq++;
	true_pos.header.stamp = timestamp;
	true_pos.header.frame_id = "groundtruth";

	float true_x = std::stof(words[1]);
	float true_y = std::stof(words[2]);
	float true_z = 0.0;
	float true_theta = std::stof(words[3]);

	true_pos.pose.position.x = true_x;
	true_pos.pose.position.y = true_y;
	true_pos.pose.position.z = true_z;
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(true_theta).normalized(), true_pos.pose.orientation);

	bag.write(node_private_name + "TRUEPOS_topic", timestamp, true_pos);

	geometry_msgs::PoseStamped odom_pos;
	true_pos.header.seq = seq++;
	true_pos.header.stamp = timestamp;
	true_pos.header.frame_id = "odom";

	float odom_x = std::stof(words[4]);
	float odom_y = std::stof(words[5]);
	float odom_z = 0.0;
	float odom_theta = std::stof(words[6]);

	odom_pos.pose.position.x = odom_x;
	odom_pos.pose.position.y = odom_y;
	odom_pos.pose.position.z = odom_z;
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(odom_theta).normalized(), odom_pos.pose.orientation);

	bag.write(node_private_name + "ODOM_topic", timestamp, odom_pos);

		
}

// CARMEN format:
// ROBOTLASER1 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode 
// num_readings [range_readings] num_remissions [remission values] laser_pose_x laser_pose_y laser_pose_theta 
// robot_pose_x robot_pose_y robot_pose_theta laser_tv laser_rv forward_safety_dist side_safty_dist turn_axis
void carmen_log_to_rosbag::publish_robotlaser_message( const std::string& to_parse )
{
	// split the string
	std::vector<std::string> words;
	split_string_by_space(to_parse, words);

	int laser_message_string_size = 10 + std::stoi(words[8]) + std::stoi(words[9 + std::stoi(words[8])]);
	std::string to_find = words[laser_message_string_size];
	std::size_t pos = to_parse.find(to_find, 0);
	// publish_rawlaser_message accepts strings where timestamps and the likes are the last 3 words
	std::string timestamp_and_location = (words[words.size() - 3]) + " " + words[words.size() - 2] 
	                                                             + " " + words[words.size() - 1];

	publish_rawlaser_message(to_parse.substr(0, pos) + timestamp_and_location);

	words = std::vector<std::string>(words.begin() + laser_message_string_size, words.end());
	std::string common_part;
	separate_with_string(common_part, {words[6], words[7], "0.000000", timestamp_and_location});

	std::string laser_odometry_message;
	separate_with_string(laser_odometry_message, {"ODOM", words[0], words[1], words[2], common_part});
	std::string robot_odometry_message;
	separate_with_string(robot_odometry_message, {"ODOM", words[3], words[4], words[5], common_part});

	publish_odometry_message(laser_odometry_message, "laser");
	publish_odometry_message(robot_odometry_message, "robot");

}	

void carmen_log_to_rosbag::param( const std::string& to_parse )
{
	std::vector<std::string> words;
	split_string_by_space(to_parse, words);

	ros::param::set(node_private_name + words[1], words[2]);
}


// CARMEN format:
// NMEAGGA gpsnr utc latitude_dm lat_orient longitude_dm long_orient gps_quality num_satellites hdop 
// sea_level alititude geo_sea_level geo_sep data_age
// OR:
// NMEARMC gpsnr validity utc latitude_dm lat_orient longitude_dm long_orient speed course variation var_dir date
// both are published with this message
void carmen_log_to_rosbag::publish_NMEA_message( const std::string& to_parse, const std::string& topic )
{
	std::vector<std::string> words;
	split_string_by_space(to_parse, words);

	nmea_msgs::Sentence nmea_sentence;

	ros::Time timestamp(std::stof(words[words.size() - 3]));
	nmea_sentence.header.stamp = timestamp;
	nmea_sentence.header.frame_id = "base_link";
	nmea_sentence.header.seq = seq++;

	separate_with_string(nmea_sentence.sentence, words.begin() + 2, words.end(), ",");

	bag.write(node_private_name + topic, timestamp, nmea_sentence);
}

