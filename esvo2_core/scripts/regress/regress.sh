#!/bin/bash
# Replay regression gate: replays the reference bags through the committed
# launch files and checks the results against fixed bounds (regress_check.py).
#
# usage: regress.sh OUT_DIR [case...]     cases: hallway2 hallway3 hallway4 mvsec1 (default: all)
# env:   DATA (default /root/datasets), needs a running roscore.
# Takes ~12 min for all four cases at 1x. Exit status 0 only if every case passes.
set -u
OUT=$1; shift
CASES=${*:-hallway2 hallway3 hallway4 mvsec1}
DATA=${DATA:-/root/datasets}
HERE=$(cd "$(dirname "$0")" && pwd)
PKG=$(cd "$HERE/../.." && pwd)
mkdir -p "$OUT"
EVK4_TOPICS="/evk4_left/events /evk4_right/events /imu/data_synced /imu/data"
EVK4_LAUNCH=$PKG/launch/system/system_evk4_mapping.launch
for c in $CASES; do
  case $c in
    hallway2) "$PKG/scripts/replay_eval.sh" "$OUT/$c" "$DATA/evk4/hallway2.bag" "$EVK4_TOPICS" "$EVK4_LAUNCH" use_sim_time:=true gui:=false ;;
    hallway3) "$PKG/scripts/replay_eval.sh" "$OUT/$c" "$DATA/evk4/hallway3.bag" "$EVK4_TOPICS" "$EVK4_LAUNCH" use_sim_time:=true gui:=false ;;
    hallway4) "$PKG/scripts/replay_eval.sh" "$OUT/$c" "$DATA/evk4/hallway4_lateral.bag" "$EVK4_TOPICS" "$EVK4_LAUNCH" use_sim_time:=true gui:=false ;;
    mvsec1)   # vision-only, as validated against the paper (ATE 7.9 cm): the stock
              # upenn configs set USE_IMU but system_upenn.launch never remaps the IMU.
              for k in mapping tracking; do
                sed 's/^USE_IMU:.*/USE_IMU: False/' "$PKG/cfg/$k/${k}_upenn_AA.yaml" > "$OUT/${k}_upenn_vision.yaml"
              done
              "$PKG/scripts/replay_eval.sh" "$OUT/$c" "$DATA/mvsec/indoor_flying1_5ms.bag" "/davis/left/events /davis/right/events" \
                "$PKG/launch/system/system_upenn.launch" gui:=false \
                mapping_cfg:="$OUT/mapping_upenn_vision.yaml" tracking_cfg:="$OUT/tracking_upenn_vision.yaml" ;;
    *) echo "unknown case $c"; exit 2 ;;
  esac
done
python3 "$HERE/regress_check.py" "$OUT" "$DATA" $CASES
