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
	//void publish_positionlaser_message( std::string to_parse ); // <-- found NO information about this output_bag_file_name
	void publish_NMEA_message( const std::string& to_parse, const std::string& topic );
	// official CARMEN logger docs do not mention those messages
	// neither does any of the RADISH datasets
	// so they are not implemented
	/*void publish_sonar_message( std::string to_parse ); // <- this is not even properly supported by CARMEN
	void publish_bumper_message( std::string to_parse );
	void publish_scanmark_message( std::string to_parse );
	void publish_IMU_message( std::string to_parse );
	void publish_vectormove_message( std::string to_parse );
	void publish_robotvelocity_message( std::string to_parse );
	void publish_followtrajectory_message( std::string to_parse );
	void publish_basevelocity_message( std::string to_parse );*/

	void param( const std::string& to_parse );
	// there is such a line in the log file but I did not manage to find any information on its usage
	// so I might add it if it's needed but I don't know what this even means as of now
	// void sync( std::string to_parse );



private:
	rosbag::Bag bag;

	std::string node_private_name; // this will be used to avoid publishing to global namespace
	int seq; 
};


#endif
