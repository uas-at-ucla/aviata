#ifndef FASTDDS_CONFIG_HPP
#define FASTDDS_CONFIG_HPP

#include <string>

#define FAST_DDS_ENV_VAR "FASTRTPS_DEFAULT_PROFILES_FILE"
#define FAST_DDS_XML_FILE_NAME "fastdds_config.xml"
#define NUM_PEERS 15
#define MESH_IP_PREFIX "10.10.0."

std::string get_mesh_ip_address();

extern std::string mesh_ip_address;

void configure_fastdds();

#endif
