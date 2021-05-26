#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>

#define FAST_DDS_ENV_VAR "FASTRTPS_DEFAULT_PROFILES_FILE"
#define FAST_DDS_XML_FILE_NAME "fastdds_config.xml"
#define NUM_PEERS 15
#define MESH_IP_PREFIX "10.10.0."

std::string get_mesh_ip_address() { // https://stackoverflow.com/a/265978/15685374
    struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;
    char addressBuffer[INET_ADDRSTRLEN] = {'\0'};
    
    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET) {
            // is a valid IPv4 Address
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            if (std::string(addressBuffer).rfind(MESH_IP_PREFIX, 0) == 0) { // if this is a mesh network IP address
                return addressBuffer;
            }
        }
    }
    if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
    return "";
}

void configure_fastdds() {
    std::string ip_address = get_mesh_ip_address();
    if (ip_address == "") { // if not on the mesh network, don't configure
        return;
    }

    std::ofstream file;
    file.open(FAST_DDS_XML_FILE_NAME, std::ofstream::trunc);

    file << "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n";
    file << "<dds>\n";
    file << "    <profiles xmlns=\"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles\">\n";
    file << "        <transport_descriptors>\n";
    file << "            <transport_descriptor>\n";
    file << "                <transport_id>udp_transport</transport_id>\n";
    file << "                <type>UDPv4</type>\n";
    file << "                <interfaceWhiteList>\n";
    file << "                    <address>" + ip_address + "</address>\n";
    file << "                </interfaceWhiteList>\n";
    file << "            </transport_descriptor>\n";
    file << "        </transport_descriptors>\n";
    file << "        \n";
    file << "        <participant profile_name=\"participant_profile_ros2\" is_default_profile=\"true\">\n";
    file << "            <rtps>\n";
    file << "                <name>profile_for_ros2_context</name>\n";
    file << "                \n";
    file << "                <userTransports>\n";
    file << "                    <transport_id>udp_transport</transport_id>\n";
    file << "                </userTransports>\n";
    file << "                <useBuiltinTransports>false</useBuiltinTransports>\n";
    file << "                \n";
    file << "                <builtin>\n";
    file << "                    <metatrafficUnicastLocatorList>\n";
    file << "                        <locator>\n";
    file << "                            <udpv4>\n";
    file << "                                <port>22223</port>\n";
    file << "                            </udpv4>\n";
    file << "                        </locator>\n";
    file << "                    </metatrafficUnicastLocatorList>\n";
    file << "                    <initialPeersList>\n";

    // Configure list of peers in absence of multicast discovery.
    for (int i = 0; i < NUM_PEERS; i++) {
        file << "                        <locator>\n";
        file << "                            <udpv4>\n";
        file << "                                <address>" MESH_IP_PREFIX + std::to_string(i) + "</address>\n";
        file << "                                <port>22223</port>\n";
        file << "                            </udpv4>\n";
        file << "                        </locator>\n";
    }

    file << "                    </initialPeersList>\n";
    file << "                </builtin>\n";
    file << "            </rtps>\n";
    file << "        </participant>\n";
    file << "    </profiles>\n";
    file << "    \n";
    file << "    <log>\n";
    file << "        <use_default>FALSE</use_default>\n";
    file << "        <consumer>\n";
    file << "            <class>StdoutConsumer</class>\n";
    file << "        </consumer>\n";
    file << "    </log>\n";
    file << "</dds>\n";

    file.close();
    
    setenv(FAST_DDS_ENV_VAR, FAST_DDS_XML_FILE_NAME, true);
}
