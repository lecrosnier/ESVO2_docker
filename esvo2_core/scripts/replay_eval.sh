#!/bin/bash
# Replay a bag through an ESVO2 launch file and record the tracked poses.
# usage: replay_eval.sh OUT_BAG BAG "TOPICS" LAUNCH_FILE [roslaunch args...]
#   OUT_BAG  poses (/esvo2_tracking/pose_pub) are recorded here; node output goes to OUT_BAG.log
#   TOPICS   space-separated topics to play from BAG
# env: PLAYRATE (default 1), PLAYARGS (extra rosbag play args, e.g. "-s 10.5")
# Needs a running roscore. Prints SGM init and tracking reset counts at the end.
# Clears the mapping/tracking/image_representation parameter namespaces before
# each run: rosparam load merges into the parameter server, so on a long-lived
# roscore, keys set by an earlier run's config would otherwise survive a later
# config that omits them.
set -u
OUT=$1; BAG=$2; TOPICS=$3; LAUNCH=$4; shift 4
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
# rosparam load merges into the parameter server, so keys from an earlier run on the same
# roscore would survive a config that omits them; start every replay from a clean slate.
for ns in /esvo2_Mapping /esvo2_Tracking /image_representation_left /image_representation_right; do
  rosparam delete "$ns" > /dev/null 2>&1
done
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
