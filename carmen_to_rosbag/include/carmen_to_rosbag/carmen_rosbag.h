#include <ros/ros.h>
#include <rosbag/bag.h>

#ifndef _CARMEN_ROSBAG_CONVERTER_PACKAGE_
#define _CARMEN_ROSBAG_CONVERTER_PACKAGE_

class carmen_log_to_rosbag
{
public:

	void convert( const char* carmen_log_file_name, const char* output_bag_file_name );

	// publish various messages
	// IMPORTANT: does not work with old formats like FLASER/RLASER 
	void publish_odometry_message( const std::string& to_parse, const std::string& add_to_topic = "" );	
	void publish_truepos_message( const std::string& to_parse );
	void publish_rawlaser_message( const std::string& to_parse );
	void publish_robotlaser_message( const std::string& to_parse );
	void publish_NMEA_message( const std::string& to_parse, const std::string& topic );
	void param( const std::string& to_parse );
	

private:
	rosbag::Bag bag;

	std::string node_private_name; // this will be used to avoid publishing to global namespace
	int seq; 
};


#endif
