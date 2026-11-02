#!/bin/bash
# Save the current slam_toolbox map (pgm/yaml + serialized pose graph) to
# /home/unitree/hu/go2/slam_maps/. Run inside the go2_humble container while a
# mapping or localization session is active.
#
#   slam_save_map.sh [name]     (default: venue)
set -e
DIR=/home/unitree/hu/go2/slam_maps
NAME="${1:-venue}"
mkdir -p "$DIR"

echo "saving pgm/yaml -> $DIR/$NAME"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$DIR/$NAME'}}"

echo "serializing pose graph -> $DIR/$NAME"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$DIR/$NAME'}"

ls -la "$DIR"
