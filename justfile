set dotenv-load # to read ROBOT_NAMESPACE from .env file

[private]
default:
    @just --list

[private]
check-husarion-webui:
    #!/bin/bash
    if ! command -v snap &> /dev/null; then
        echo "Snap is not installed. Please install Snap first and try again."
        echo "sudo apt install snapd"
        exit 1
    fi

    if ! snap list husarion-webui &> /dev/null; then
        echo "husarion-webui is not installed."
        read -p "Do you want to install husarion-webui? (y/n): " choice
        case "$choice" in
            y|Y )
                sudo snap install husarion-webui --channel=jazzy
                ;;
            n|N )
                echo "Installation aborted."
                exit 0
                ;;
            * )
                echo "Invalid input. Please respond with 'y' or 'n'."
                exit 1
                ;;
        esac
    fi

# Start navigation on User Computer inside Husarion UGV
start-hardware service="navigation docking":
    #!/bin/bash
    docker compose -f compose.hardware.yaml down
    docker compose -f compose.hardware.yaml pull
    docker compose -f compose.hardware.yaml up {{service}}


# Start Gazebo simulator with full autonomy stack
start-simulation:
    #!/bin/bash
    xhost +local:docker
    docker compose -f compose.simulation.yaml down
    docker compose -f compose.simulation.yaml pull
    docker compose -f compose.simulation.yaml up

# Configure and run Husarion WebUI
start-visualization: check-husarion-webui
    #!/bin/bash
    sudo cp config/foxglove-layout.json /var/snap/husarion-webui/common/foxglove-husarion-ugv-navigation.json
    sudo snap set husarion-webui webui.layout=husarion-ugv-navigation
    sudo snap set husarion-webui ros.namespace=panther
    sudo snap set husarion-webui ros.transport=rmw_cyclonedds_cpp
    sudo husarion-webui.start

    local_ip=$(hostname -I | awk '{print $1}')
    hostname=$(hostname)
    echo "Open a web browser and go to http://$local_ip:8080/ui or http://$hostname:8080/ui if your device is connected to the same Husarnet network."

# Stop Husarion WebUI
stop-visualization: check-husarion-webui
    #!/bin/bash
    sudo husarion-webui.stop

# Dock Husarion UGV to the charging dock using navigation stack
dock DOCK_NAME:
    #!/bin/bash
    docker compose -f compose.simulation.yaml exec docking bash -c \
     "source install/setup.bash && ros2 action send_goal /panther/dock_robot nav2_msgs/action/DockRobot \" {  dock_type: charging_dock, navigate_to_staging_pose: true, dock_id: {{DOCK_NAME}} }\""

# Dock Husarion UGV to the charging dock without using navigation stack
dock-direct DOCK_NAME:
    #!/bin/bash
    docker compose -f compose.simulation.yaml exec docking bash -c \
     "source install/setup.bash && ros2 action send_goal /panther/dock_robot nav2_msgs/action/DockRobot \" {  dock_type: charging_dock, navigate_to_staging_pose: false, dock_id: {{DOCK_NAME}} }\""


# Undock Husarion UGV from the charging dock
undock:
    #!/bin/bash
    docker compose -f compose.simulation.yaml exec docking bash -c \
     "source install/setup.bash && ros2 action send_goal /panther/undock_robot nav2_msgs/action/UndockRobot \" {  dock_type: charging_dock }\""

setup-os:
    bash setup_os.sh

# Map-nav (demo-mapnav.md): Nav2 behind the airlock of Husarion UGV OS, with an Ouster. Set ROBOT_NAMESPACE first.
mapnav-up:
    docker compose -f compose.mapnav.yaml --profile ouster up -d

# Start the map-nav stack with the Ouster and the OAK's video for the Cockpit (demo-mapnav.md §7)
mapnav-oak-up:
    docker compose -f compose.mapnav.yaml --profile ouster --profile oak up -d

# Start the map-nav stack with a Slamtec RPLIDAR S3 instead of the Ouster (demo-mapnav.md §8; not tested on a robot yet)
mapnav-rplidar-up:
    docker compose -f compose.mapnav.yaml --profile rplidar up -d

# Stop the map-nav stack, whichever LiDAR and whether or not the OAK's service runs
mapnav-down:
    docker compose -f compose.mapnav.yaml --profile ouster --profile rplidar --profile oak down

# Follow the map-nav stack's logs
mapnav-logs:
    docker compose -f compose.mapnav.yaml --profile ouster --profile rplidar --profile oak logs -f
