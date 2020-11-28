#include "drone.hpp"
#include "px4_io.hpp"

drone::drone()
{
	//TODO: default constructor?
}

drone::drone(std::string connection_url)
{
	std::shared_ptr<System> system = connect_to_pixhawk(connection_url);
}

