#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <carmen_to_rosbag/carmen_rosbag.h>

bool check_file_existence( const char* filename )
{
	std::ifstream in(filename);
	return in.good();
}

int main( int argc, char** argv )
{

	if (argc < 5)
	{
		std::cout << "Usage: rosrun carmen_to_rosbag carmen_to_rosbag_node -i path_to_carmen_file -o path_to_bag_file.\n";
		return 0;
	} 

    char** p1 = std::find(argv, argv + argc, std::string("-i"));
    // assuming that the "-i" should be immediately followed by the filename    
    if (p1 - argv >= (argc - 1) || !check_file_existence(*(p1 + 1)))
    {
        std::cout << "No input file provided.\n";
        return 2;
    }
    char** p2 = std::find(argv, argv + argc, std::string("-o"));
    if (p2 == argv + argc || p2 == argv + argc - 1)
    {
        std::cout << "No output file provided.\n";
        return 2;
    }


	ros::init(argc, argv, "converter");
	carmen_log_to_rosbag converter;
	converter.convert(*(p1 + 1), *(p2 + 1));
	
	return 0;
}