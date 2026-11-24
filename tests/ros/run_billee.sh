#!/bin/bash
# Runs inside the private BILLEE stack. Started by scripts/billee_check.py.
SOFTWARE=/workspaces/billee-software-2027/ros2_ws
PREFIX=$SOFTWARE/.pixi/envs/default
export CONDA_PREFIX=$PREFIX PATH=$PREFIX/bin:$PATH ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=85 HOME=/tmp/case-home
mkdir -p "$HOME" /evidence
for script in "$PREFIX"/etc/conda/activate.d/*.sh; do
    case "$script" in *binutils*|*gcc*|*gfortran*|*gxx*) ;; *) . "$script" ;; esac
done
. "$SOFTWARE/install/local_setup.bash"
# The team's roboclaw package builds JetsonGPIO but does not install it.
export LD_LIBRARY_PATH="$SOFTWARE/build/roboclaw_hardware_interface/jetson_gpio:${LD_LIBRARY_PATH:-}"
STATUS=0

stop_all() {
    pkill -INT -f 'arm_moveit_bridge' 2>/dev/null
    sleep 0.5
    pkill -KILL -f 'move_group|arm_moveit_bridge|ros2_control_node|ign gazebo|ruby|parameter_bridge|robot_state_publisher|joint_state_publisher|spawner|ekf_node|foxglove|ros2 launch|synthetic|status_light|controller_server|planner_server|lifecycle_manager' 2>/dev/null
    sleep 2
}
trap stop_all EXIT

build() {  # build <workspace> <packages...>
    local ws=$1
    shift
    mkdir -p "$ws/src"
    (cd "$ws" && colcon build --packages-select "$@" >"/evidence/$(basename "$ws").build.log" 2>&1)
}

stock() {
    mkdir -p /tmp/ws_stock/src
    cp -r /cases/stock/robot_description /tmp/ws_stock/src/
    build /tmp/ws_stock robot_description || { echo "CHECK_FAIL stock package builds"; return 1; }
    . /tmp/ws_stock/install/local_setup.bash
    ros2 launch chassis_bringup rover.launch.py $ROVER_ARGS joy_control:=false arm_camera:=false >/evidence/stock.log 2>&1 &
    sleep 9
    timeout 115 python /toolkit/tests/ros/check_stack.py | sed 's/^RESULT /STOCK /; s/^METRIC /STOCK_METRIC /'
    stop_all
}

rover() {
    mkdir -p /tmp/ws_rover/src
    cp -r /cases/rover/robot_description /tmp/ws_rover/src/
    build /tmp/ws_rover robot_description || { echo "CHECK_FAIL rover export builds"; return 1; }
    . /tmp/ws_rover/install/local_setup.bash
    ros2 launch chassis_bringup rover.launch.py $ROVER_ARGS joy_control:=false arm_camera:=false >/evidence/rover.log 2>&1 &
    sleep 9
    timeout 115 python /toolkit/tests/ros/check_stack.py | sed 's/^RESULT /ROVER /; s/^METRIC /ROVER_METRIC /'
    stop_all
}

assembly() {
    mkdir -p /tmp/ws_all/src
    cp -r /cases/assembly/robot_description /cases/assembly/arm_description /cases/assembly/billee_arm_moveit_config /tmp/ws_all/src/
    mkdir -p /tmp/ws_all/src/arm_bringup
    cp -r "$SOFTWARE/src/arm_bringup/config" /tmp/ws_all/src/arm_bringup/config
    touch /tmp/ws_all/src/arm_bringup/COLCON_IGNORE
    build /tmp/ws_all robot_description arm_description billee_arm_moveit_config \
        && echo "CHECK_PASS rover, arm, and MoveIt packages build" || { echo "CHECK_FAIL packages build"; tail -12 /evidence/ws_all.build.log; return 1; }
    . /tmp/ws_all/install/local_setup.bash
    (cd /tmp/ws_all/src/arm_description && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q -p no:cacheprovider test/test_import_urdf.py >/evidence/arm_tests.log 2>&1) \
        && echo "CHECK_PASS team's arm_description tests pass on the generated model" \
        || { echo "CHECK_FAIL team's arm_description tests"; tail -12 /evidence/arm_tests.log; STATUS=1; }
    ros2 launch chassis_bringup rover.launch.py $ARM_ROVER_ARGS joy_control:=false arm_camera:=false >/evidence/assembly_rover.log 2>&1 &
    sleep 14
    ros2 launch arm_bringup arm.launch.py can_interface:=mock roboclaw_port:=mock >/evidence/assembly_arm.log 2>&1 &
    sleep 6
    ros2 launch billee_arm_moveit_config moveit.launch.py >/evidence/moveit_refused.log 2>&1 &
    MOVEIT=$!
    timeout 130 python /toolkit/tests/ros/arm_moveit_check.py refused || STATUS=1
    kill -INT "$MOVEIT" 2>/dev/null
    sleep 2
    pkill -INT -f arm_moveit_bridge 2>/dev/null
    sleep 1
    pkill -KILL -f 'move_group|arm_moveit_bridge' 2>/dev/null
    sleep 2
    ros2 launch billee_arm_moveit_config moveit.launch.py allow_unverified:=true >/evidence/moveit.log 2>&1 &
    timeout 240 python /toolkit/tests/ros/arm_moveit_check.py assembly || STATUS=1
    stop_all
    return "$STATUS"
}

standalone() {
    mkdir -p /tmp/ws_arm/src
    cp -r /cases/arm/arm_description /cases/arm/billee_arm_moveit_config /tmp/ws_arm/src/
    build /tmp/ws_arm arm_description billee_arm_moveit_config \
        && echo "CHECK_PASS arm-only packages build" || { echo "CHECK_FAIL arm-only packages build"; return 1; }
    . /tmp/ws_arm/install/local_setup.bash
    ros2 launch arm_bringup arm.launch.py can_interface:=mock roboclaw_port:=mock >/evidence/standalone_arm.log 2>&1 &
    sleep 5
    ros2 launch billee_arm_moveit_config moveit.launch.py allow_unverified:=true >/evidence/standalone.log 2>&1 &
    timeout 110 python /toolkit/tests/ros/arm_moveit_check.py standalone
    local result=$?
    stop_all
    return "$result"
}

export ROVER_ARGS ARM_ROVER_ARGS
ROVER_ARGS=$(cat /cases/rover_args.txt)
ARM_ROVER_ARGS=$(cat /cases/arm_rover_args.txt)
case "${1:-all}" in
    rover) ( stock ) || STATUS=1; ( rover ) || STATUS=1 ;;
    arm) ( assembly ) || STATUS=1; ( standalone ) || STATUS=1 ;;
    *) ( stock ) || STATUS=1; ( rover ) || STATUS=1; ( assembly ) || STATUS=1; ( standalone ) || STATUS=1 ;;
esac
trap - EXIT
exit "$STATUS"
