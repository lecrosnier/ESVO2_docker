#!/bin/bash
# Replay a bag through an ESVO2 launch file and record the tracked poses.
# usage: replay_eval.sh OUT_BAG BAG "TOPICS" LAUNCH_FILE [roslaunch args...]
#   OUT_BAG  poses (/esvo2_tracking/pose_pub) are recorded here; node output goes to OUT_BAG.log
#   TOPICS   space-separated topics to play from BAG
# env: PLAYRATE (default 1), PLAYARGS (extra rosbag play args, e.g. "-s 10.5")
# Needs a running roscore. Prints SGM init and tracking reset counts at the end.
set -u
OUT=$1; BAG=$2; TOPICS=$3; LAUNCH=$4; shift 4
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
roslaunch --wait "$LAUNCH" "$@" > "$OUT.log" 2>&1 &
LPID=$!
sleep 6
rosbag record -O "$OUT" /esvo2_tracking/pose_pub __name:=replay_eval_record > /dev/null 2>&1 &
RPID=$!
sleep 1
rosbag play --clock -q -r "${PLAYRATE:-1}" ${PLAYARGS:-} "$BAG" --topics $TOPICS
sleep 2
kill -INT $RPID; wait $RPID 2>/dev/null
kill -INT $LPID; wait $LPID 2>/dev/null
echo "SGM inits: $(grep -c 'Initialization (SGM)' "$OUT.log"), tracking resets: $(grep -c 're-initialized' "$OUT.log")"
