#!/bin/bash
# Cycle-test the gripper: alternate close/open until Ctrl-C.
# Usage:  ./cycle_test.sh [controller] [joint] [close_rad] [open_rad] [force_g] [period_s]
# Example: ./cycle_test.sh left_gripper_controller left_gripper_joint1 0.8663 0.0 300 3

set -euo pipefail

CTRL=${1:-left_gripper_controller}
JOINT=${2:-left_gripper_joint1}
CLOSE=${3:-0.8663}
OPEN=${4:-0.0}
FORCE=${5:-300.0}
PERIOD=${6:-3}

echo "Cycling /${CTRL} on joint=${JOINT}: close=${CLOSE} open=${OPEN} force=${FORCE}g period=${PERIOD}s"
echo "Ctrl-C to stop."

send() {
  local pos=$1 eff=$2
  ros2 action send_goal "/${CTRL}/gripper_cmd" \
    control_msgs/action/ParallelGripperCommand \
    "{command: {name: ['${JOINT}'], position: [${pos}], effort: [${eff}]}}" \
    > /dev/null
  printf "[%s] pos=%s eff=%s\n" "$(date +%H:%M:%S)" "$pos" "$eff"
}

while true; do
  send "$CLOSE" "$FORCE"
  sleep "$PERIOD"
  send "$OPEN"  "0.0"
  sleep "$PERIOD"
done
