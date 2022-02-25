'''
use_guide () {
    echo "Usage: ./script.sh [options]"
    echo "Options:"
    echo "  -i: drone id"
    echo "  -s: state (standby, follower, leader)"
    echo "  -d: docking slot"
    echo "  -c: connection url"
    echo "  -f: airframe in use ('4', '4_alt', or '2_sim')"
    echo "  -m: max-missing"
    echo "  -w: network name the Rpi connects to"
    echo "  -p: password to the network"
}
'''

INPUTARGS=""

while getopts i:s:d:c:n:m:w:p: flag
do
    case "${flag}" in
        i) id=${OPTARG}
            INPUTARGS+="--id $id "
        ;;
        s) state=${OPTARG}
            INPUTARGS+="--state $state "
        ;;
        d) docking_slot=${OPTARG}
            INPUTARGS+="--docking-slot $docking_slot "
        ;;
        c) connection_url=${OPTARG}
            INPUTARGS+="--connection-url $connection_url "
        ;;
        n) frame=${OPTARG}
            INPUTARGS+="--frame $frame "
        ;;
        m) max_missing=${OPTARG}
            INPUTARGS+="--max-missing $max_missing "
        ;;
        w) network=${OPTARG};;
        p) password=${OPTARG};;
    esac
done

# Edit /etc/wpa_supplicant/wpa_supplicant.conf
# Replace the line starting with ssid with ssid="{network name}", where {network name} is the name of the network we will connect the raspberry pi to
sed -i "/ssid/c\ssid=$network" /etc/wpa_supplicant/wpa_supplicant.conf
# Replace the line starting with psk with psk="{network password}, where {network password} is the password of the network we will connect to
sed -i "/psk/c\psk=$password" /etc/wpa_supplicant/wpa_supplicant.conf
# Restart wifi service
wpa_cli -i wlan0 reconfigure
# Kill aviata_drone process if it is currently running
killall aviata_drone
# Remove output file from build folder if it currently exists
rm aviata_drone.out
# Navigate to correct folder
cd ~/aviata/
# Pull from origin
git pull origin
# Source ROS2 startup script
chmod +x rpi_tools/env_setup.sh && ./rpi_tools/env_setup.sh
# Build most recent code
cd controls
mkdir build && cd build
cmake ..
make
# Edit most recent crontab
@reboot bash -c "export PATH="$PATH:/usr/sbin" && cd ~/aviata/controls/build && source ../../rpi_tools/env_setup.sh && $sh && ../../rpi_tools/wait_for_mesh_network.sh && ./aviata_drone $INPUTARGS &>aviata_drone.out &"
# Reboot the Rpi
sudo reboot
# Concatenate and return the output file from build folder (aviata_drone.out I think)
cat ~/aviata/controls/build/aviata_drone.out    