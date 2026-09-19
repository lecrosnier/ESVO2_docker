#!/bin/bash
# Replay a bag through an ESVO2 launch file and record the tracked poses.
# usage: replay_eval.sh OUT_BAG BAG "TOPICS" LAUNCH_FILE [roslaunch args...]
#   OUT_BAG  poses (/esvo2_tracking/pose_pub) are recorded here; node output goes to OUT_BAG.log
#   TOPICS   space-separated topics to play from BAG
# env: PLAYRATE (default 1), PLAYARGS (extra rosbag play args, e.g. "-s 10.5")
# Needs a running roscore. Prints SGM init and tracking reset counts at the end.
# Clears the mapping/tracking/image_representation parameter namespaces before each run:
# rosparam load merges into the parameter server, so on a long-lived roscore, keys set by
# an earlier run's config would otherwise survive a later config that omits them. (The
# EVK4/upenn system launch files also set clear_params="true" on these nodes, so this only
# matters for other launch files or params set outside the node namespaces.)
set -u
OUT=$1; BAG=$2; TOPICS=$3; LAUNCH=$4; shift 4
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
for ns in /esvo2_Mapping /esvo2_Tracking /image_representation_left /image_representation_right; do
  rosparam delete "$ns" > /dev/null 2>&1
done
roslaunch --wait "$LAUNCH" "$@" > "$OUT.log" 2>&1 &
LPID=$!
# If this script is interrupted (Ctrl-C, or killed) before the normal shutdown below,
# still send INT to the record/launch processes instead of leaving them running.
trap 'kill -INT ${RPID:-} ${LPID:-} 2>/dev/null' INT TERM
sleep 6
rosbag record -O "$OUT" /esvo2_tracking/pose_pub __name:=replay_eval_record > /dev/null 2>&1 &
RPID=$!
sleep 1
rosbag play --clock -q -r "${PLAYRATE:-1}" ${PLAYARGS:-} "$BAG" --topics $TOPICS
sleep 2
kill -INT $RPID; wait $RPID 2>/dev/null
kill -INT $LPID; wait $LPID 2>/dev/null
rosparam set /use_sim_time false
echo "SGM inits: $(grep -c 'Initialization (SGM)' "$OUT.log"), tracking resets: $(grep -c 're-initialized' "$OUT.log")"
