#include "lib/drone.hpp"
#include "lib/util.hpp"
#include <unistd.h>
#include <algorithm>

int print_use_guide() {
    std::cout << "Usage: ./aviata_drone [options]" << std::endl;
    std::cout << "Options: " << std::endl;
    std::cout << "  --id (-i): Drone ID, defaults to hostname with rpi- stripped" << std::endl;
    std::cout << "  --state (-s): Initial state (standby, follower, leader), defaults to standby" << std::endl;
    std::cout << "  --docking-slot (-d): Docking slot" << std::endl;
    std::cout << "  --connection-url (-c): Defaults to RPi serial port" << std::endl;
    std::cout << "  --num-drones (-n): Total number of drones in the frame, defaults to 4" << std::endl;
    std::cout << "  --max-missing (-m): Number of drones in the frame missing to still fly, defaults to 0" << std::endl;
    std::cout << "  With no arguments, this starts a drone in STANDBY with ID based on the machine hostname, and connects to the RPi serial port." << std::endl;
    return EINVAL;
}

int main(int argc, char** argv) {
    if (argc >= 2 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        print_use_guide();
        return 0;
    }

    SetupOptions opt;

    char hostname[256] = "";
    gethostname(hostname, sizeof(hostname));
    opt.drone_id = hostname;
    
    opt.drone_id = "adbad";
    opt.state = STANDBY;
    opt.docking_slot = -1;
    opt.connection_url = "serial:///dev/ttyAMA0:921600";
    opt.frame = AVIATA_4;
    opt.max_missing_drones = -1;

    char* curr_option = nullptr;
    for (int i = 1; i < argc; i++) {
        if (strlen(argv[i]) > 0) {
            if (argv[i][0] == '-') {
                if (curr_option != nullptr) {
                    return print_use_guide();
                }
                else {
                    curr_option = argv[i];
                }
            }
            else {
                if (curr_option == nullptr) {
                    return print_use_guide();
                }
                if (strcmp(curr_option, "-i") == 0 || strcmp(curr_option, "--id") == 0) {
                    opt.drone_id = argv[i];
                }
                else if (strcmp(curr_option, "-s") == 0 || strcmp(curr_option, "--state") == 0) {
                    if (strcmp(argv[i], "follower") == 0) {
                        opt.state = DOCKED_FOLLOWER;
                    }
                    else if (strcmp(argv[i], "leader") == 0) {
                        opt.state = DOCKED_LEADER;
                    }
                    else if (strcmp(argv[i], "standby") == 0) {
                        opt.state = STANDBY;
                    }
                    else {
                        std::cout << "Invalid initial state: " << argv[i] << std::endl;
                        return EINVAL;
                    }
                }
                else if (strcmp(curr_option, "-d") == 0 || strcmp(curr_option, "--docking-slot") == 0) {
                    opt.docking_slot = atoi(argv[i]);
                }
                else if (strcmp(curr_option, "-c") == 0 || strcmp(curr_option, "--connection-url") == 0) {
                    opt.connection_url = argv[i];
                }
                else if (strcmp(curr_option, "-f") == 0 || strcmp(curr_option, "--frame") == 0) {
                    if (strcmp(argv[i], "4") == 0) {
                        opt.frame = AVIATA_4;
                    }
                    else if (strcmp(argv[i], "4_alt") == 0) {
                        opt.frame = AVIATA_4_ALT;
                    }
                    else {
                        std::cout << "Invalid frame: " << argv[i] << std::endl;
                        return EINVAL;
                    }
                }
                else if (strcmp(curr_option, "-m") == 0 || strcmp(curr_option, "--max-missing") == 0) {
                    opt.max_missing_drones = atoi(argv[i]);
                }
                else {
                    return print_use_guide();
                }

                curr_option = nullptr;
            }
        }
        else {
            return print_use_guide();
        }

    }

    if (opt.drone_id.rfind("rpi-", 0) == 0) { // if hostname starts with "rpi-"
        opt.drone_id = opt.drone_id.substr(4);
    }
    std::replace(opt.drone_id.begin(), opt.drone_id.end(), '-', '_');

    DroneSettings drone_settings;

    drone_settings.frame = opt.frame;
    if (opt.max_missing_drones != -1) {
        drone_settings.max_missing_drones = opt.max_missing_drones;
    } else {
        drone_settings.max_missing_drones = _config_aviata_frame_info[drone_settings.frame].max_missing_drones;
    }

    if (opt.connection_url.rfind("udp://", 0) == 0) { // if connection_url starts with "udp://"
        // If using UDP, assume we're in the simulator that does not support physical docking.
        drone_settings.sim = true;
        drone_settings.modify_px4_mixers = false;
    }
    else {
        drone_settings.sim = false;
        drone_settings.modify_px4_mixers = true;
    }

    std::cout<< "Running aviata_drone for drone " << opt.drone_id<< " with settings: "<< std::endl;
    std::cout<< "Docking slot: "<< +opt.docking_slot << " Initial state: " << opt.state << std::endl;
    std::cout<< "Total drones: "<< +_config_aviata_frame_info[drone_settings.frame].num_drones << " Maximum missing drones: " << +drone_settings.max_missing_drones << std::endl;
    std::cout<< "Connection url: " << opt.connection_url << std:: endl;

    Target t;

    Drone drone(opt.drone_id, drone_settings, t);

    if (!drone.init(opt.state, opt.docking_slot, opt.connection_url)) {
        return 1;
    }

    drone.run();

    return 0;
}
