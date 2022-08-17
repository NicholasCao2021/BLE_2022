#!/bin/bash
if [[ -f "/etc/starling/vehicle.config" ]]; then
    echo "Sourcing local vehicle configuration"
    source "/etc/starling/vehicle.config"
fi

if [[ -f "/ros_ws/install/setup.bash" ]]; then
    echo "Sourcing local install setup.bash"
    source "/ros_ws/install/setup.bash"
fi

if [ ! -v $VEHICLE_MAVLINK_SYSID ]; then
    export VEHICLE_MAVLINK_SYSID=$VEHICLE_MAVLINK_SYSID
    echo "VEHICLE_MAVLINK_SYSID setting to $VEHICLE_MAVLINK_SYSID"
else
    export VEHICLE_MAVLINK_SYSID=1
    echo "VEHICLE_MAVLINK_SYSID not set, default to 1"
fi

if [ ! -v $OFFBOARD ]; then
    echo "Running Offboard Controller"
    ros2 run bluetooth_python_node controller
else
    echo "Running Onboard Controller"
    ros2 launch bluetooth_cpp_node.launch.xml
    # ros2 launch bluetooth_dongle_cpp_node bluetooth_dongle_cpp_node.launch.xml
fi
