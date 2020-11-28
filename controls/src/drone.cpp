#include "drone.hpp"
#include "px4_io.hpp"

drone::drone()
{
	//TODO: default constructor?
}

drone::drone(std::String connection_url)
{
	system = connect_to_pixhawk(connection_url);
}

