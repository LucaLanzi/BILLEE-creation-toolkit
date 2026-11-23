#!/bin/bash
# Runs inside the BILLEE ROS 2 Humble environment. Started by scripts/ros_gates.py.
# Usage: run_gates.sh <package folder> <toolkit folder> <output folder>
set -u

SOURCE=$1
TOOLKIT=$2
OUTPUT=$3
PACKAGE=$(basename "$SOURCE")
ENV_PREFIX=${BILLEE_ROS_PREFIX:-/workspaces/billee-software-2027/ros2_ws/.pixi/envs/default}
WS="/tmp/ros ws with spaces"
FAILED=0

export CONDA_PREFIX="$ENV_PREFIX"
export PATH="$ENV_PREFIX/bin:$PATH"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=77
export HOME=/tmp/gate-home
mkdir -p "$HOME" "$OUTPUT"
set +u
for script in "$ENV_PREFIX"/etc/conda/activate.d/*.sh; do
    case "$script" in *binutils*|*gcc*|*gfortran*|*gxx*) ;; *) . "$script" ;; esac
done
set -u

gate() {
    name=$1
    shift
    log="$OUTPUT/$(echo "$name" | tr ' /' '__').log"
    if "$@" >"$log" 2>&1; then
        echo "GATE_PASS $name"
    else
        echo "GATE_FAIL $name (see $(basename "$log"))"
        tail -n 25 "$log"
        FAILED=1
    fi
}

stop_all() {
    for pid in $(jobs -p); do kill -INT "$pid" 2>/dev/null; done
    sleep 2
    for pid in $(jobs -p); do kill -KILL "$pid" 2>/dev/null; done
    pkill -KILL -f 'ign gazebo|ruby.*ign|parameter_bridge|robot_state_publisher|spawner|ros2 launch' 2>/dev/null
    sleep 1
}
trap stop_all EXIT

echo "ROS_DISTRO=${ROS_DISTRO:-unset} RMW=${RMW_IMPLEMENTATION:-default}"
echo "xacro $(ls "$ENV_PREFIX"/conda-meta | grep -m1 '^ros-humble-xacro-' | cut -d- -f4)," \
     "Gazebo $(ign gazebo --versions 2>/dev/null | head -n 1)"

mkdir -p "$WS/src"
cp -r "$SOURCE" "$WS/src/$PACKAGE"
build() { cd "$WS" && colcon build --packages-select "$PACKAGE" --event-handlers console_direct+; }
gate "package builds in a path with spaces" build
if [ ! -f "$WS/install/setup.bash" ]; then
    echo "GATE_FAIL package install tree missing"
    exit 1
fi
set +u
. "$WS/install/setup.bash"
set -u
SHARE=$(ros2 pkg prefix --share "$PACKAGE")

expanded="$OUTPUT/expanded"
mkdir -p "$expanded"
expand_all() {
    for file in "$SHARE"/urdf/*.urdf.xacro; do
        xacro "$file" -o "$expanded/$(basename "$file" .xacro)" || return 1
    done
}
gate "1 real xacro expands every model" expand_all

check_all() {
    for file in "$expanded"/*.urdf "$SHARE"/urdf/*.urdf; do
        check_urdf "$file" || return 1
    done
}
gate "2 urdfdom parses every model" check_all
PLAIN=$(ls "$SHARE"/urdf/*.urdf | head -n 1)
gate "2 expanded Xacro equals plain URDF" python "$TOOLKIT/tests/ros/compare_urdf.py" "$PLAIN" "$expanded/assembly.urdf"

ROOT=$(python -c "
import sys, xml.etree.ElementTree as ET
r = ET.parse(sys.argv[1]).getroot()
c = {j.find('child').get('link') for j in r.findall('joint')}
print(next(l.get('name') for l in r.findall('link') if l.get('name') not in c))" "$PLAIN")

python - "$PLAIN" "$OUTPUT/rsp.yaml" <<'EOF'
import json, sys
text = open(sys.argv[1], encoding="utf-8").read()
open(sys.argv[2], "w", encoding="utf-8").write(
    "robot_state_publisher:\n  ros__parameters:\n    robot_description: " + json.dumps(text) + "\n")
EOF
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$OUTPUT/rsp.yaml" >"$OUTPUT/rsp.log" 2>&1 &
ros2 run joint_state_publisher joint_state_publisher "$PLAIN" >"$OUTPUT/jsp.log" 2>&1 &
sleep 4
gate "3 published TF matches URDF kinematics" python "$TOOLKIT/tests/ros/tf_check.py" "$PLAIN" "$ROOT" arm_tool_frame
stop_all

ros2 launch "$PACKAGE" gazebo.launch.py gui:=false >"$OUTPUT/gazebo.log" 2>&1 &
ROBOT=$(basename "$PLAIN" .urdf)
gate "5 Gazebo spawn, dynamics, and control" timeout 200 python "$TOOLKIT/tests/ros/sim_check.py" "$ROBOT"
cat "$OUTPUT/5_Gazebo_spawn,_dynamics,_and_control.log" 2>/dev/null | grep -E '^SIM_'
stop_all
trap - EXIT
exit "$FAILED"
