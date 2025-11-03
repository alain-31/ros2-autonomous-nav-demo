#!/bin/bash
MAP_DIR=/home/alain/bumperbot_ws/map
MAP_BASENAME=playground_post_3_hd
PG_FILE="${MAP_DIR}/${MAP_BASENAME}"

echo "saving .yaml .pgm ..."
ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_BASENAME}" \
  --occ 0.65 --free 0.25 \
  --ros-args \
  -r map:=/map \
  -p map_subscribe_transient_local:=true
  -p save_map_timeout:=10.0

echo "done"
echo "saving .posegraph ..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: \"$PG_FILE\"}"
echo "done"

echo "checking ..."
echo "expect: .data  .pgm   .posegraph .yaml"
ls -1 "$MAP_DIR/${MAP_BASENAME}."*
echo "done"
echo "expect: .yaml  .pgm  .posegraph"
