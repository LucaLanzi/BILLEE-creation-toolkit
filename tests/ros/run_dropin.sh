#!/bin/bash
# Runs inside the BILLEE environment. Started by scripts/dropin_check.py.
# Usage: run_dropin.sh <stock|dropin>
CASE=$1
SOFTWARE=/workspaces/billee-software-2027/ros2_ws
PREFIX=$SOFTWARE/.pixi/envs/default
export CONDA_PREFIX=$PREFIX PATH=$PREFIX/bin:$PATH ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=88 HOME=/tmp/case-home
mkdir -p "$HOME" /evidence
for script in "$PREFIX"/etc/conda/activate.d/*.sh; do
    case "$script" in *binutils*|*gcc*|*gfortran*|*gxx*) ;; *) . "$script" ;; esac
done
. "$SOFTWARE/install/local_setup.bash"

# Equivalent to replacing ros2_ws/src/robot_description and rebuilding that package.
mkdir -p /tmp/ws/src
cp -r "/cases/$CASE/sim/robot_description" /tmp/ws/src/
(cd /tmp/ws && colcon build --packages-select robot_description >"/evidence/$CASE.build.log" 2>&1) \
    && echo "RESULT package_builds=True" || { echo "RESULT package_builds=False"; exit 1; }
. /tmp/ws/install/local_setup.bash

FULL="/cases/$CASE/full/robot_description/urdf/robot.urdf.xacro"
xacro "$FULL" use_sim:=true -o "/evidence/$CASE.sim.urdf" && xacro "$FULL" use_sim:=false -o "/evidence/$CASE.real.urdf" \
    && check_urdf "/evidence/$CASE.sim.urdf" >/dev/null && check_urdf "/evidence/$CASE.real.urdf" >/dev/null \
    && echo "RESULT xacro_and_urdf_valid=True" || echo "RESULT xacro_and_urdf_valid=False"

ign sdf -p "/evidence/$CASE.sim.urdf" >"/evidence/$CASE.sdf" 2>"/evidence/$CASE.sdf.err"
for marker in "sensor name='zed2i'" "ign_ros2_control::IgnitionROS2ControlPlugin" "OdometryPublisher" "systems::Imu"; do
    grep -q "$marker" "/evidence/$CASE.sdf" && echo "RESULT sdf_has_${marker##*:}=True" || echo "RESULT sdf_has_${marker##*:}=False"
done

timeout -s INT 18 ros2 launch chassis_bringup real.launch.py can_interface:=mock >"/evidence/$CASE.real.log" 2>&1
if grep -q 'Caught exception in launch' "/evidence/$CASE.real.log"; then
    echo "RESULT real_launch_starts=False"
    grep -m1 'Caught exception' "/evidence/$CASE.real.log" | cut -c1-180
else
    echo "RESULT real_launch_starts=True"
fi
# can_interface:=mock has no working driver path, so this only proves the model reaches hardware loading.
grep -q "Loading hardware 'Robot'" "/evidence/$CASE.real.log" && echo "RESULT real_reaches_hardware_load=True" \
    || echo "RESULT real_reaches_hardware_load=False"
pkill -KILL -f 'ros2_control_node|robot_state_publisher|spawner|ros2 launch' 2>/dev/null
sleep 2

ros2 launch chassis_bringup rover.launch.py mode:=sim joy_control:=false arm_camera:=false >"/evidence/$CASE.sim.log" 2>&1 &
sleep 8
timeout 110 python /toolkit/tests/ros/check_stack.py
pkill -KILL -f 'ign gazebo|ruby|parameter_bridge|robot_state_publisher|spawner|ekf_node|foxglove|ros2 launch|synthetic' 2>/dev/null
exit 0
