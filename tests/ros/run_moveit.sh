#!/bin/bash
# Runs inside the BILLEE environment with the MoveIt layer. Started by scripts/moveit_check.py.
SOFTWARE=/workspaces/billee-software-2027/ros2_ws
PREFIX=$SOFTWARE/.pixi/envs/default
export CONDA_PREFIX=$PREFIX PATH=$PREFIX/bin:$PATH ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=89 HOME=/tmp/case-home
mkdir -p "$HOME" /evidence
for script in "$PREFIX"/etc/conda/activate.d/*.sh; do
    case "$script" in *binutils*|*gcc*|*gfortran*|*gxx*) ;; *) . "$script" ;; esac
done
. "$SOFTWARE/install/local_setup.bash"

stop_all() {
    pkill -KILL -f 'move_group|ros2_control_node|ign gazebo|ruby|parameter_bridge|robot_state_publisher|spawner|ekf_node|foxglove|ros2 launch|synthetic' 2>/dev/null
    sleep 2
}
trap stop_all EXIT
STATUS=0

assembly() {
    mkdir -p /tmp/ws/src
    cp -r /cases/robot_description /cases/billee_arm_moveit_config /tmp/ws/src/
    (cd /tmp/ws && colcon build --packages-select robot_description billee_arm_moveit_config >/evidence/build.log 2>&1) \
        && echo "MOVEIT_PASS both packages build" || { echo "MOVEIT_FAIL both packages build"; tail -15 /evidence/build.log; return 1; }
    . /tmp/ws/install/local_setup.bash
    xacro "$SOFTWARE/src/robot_description/urdf/robot.urdf.xacro" use_sim:=false -o /evidence/stock.real.urdf
    xacro /cases/robot_description/urdf/robot.urdf.xacro use_sim:=false -o /evidence/assembly.real.urdf
    python - <<'EOF'
import xml.etree.ElementTree as ET
def blocks(path):
    return [[(e.tag, tuple(sorted(e.attrib.items())), (e.text or "").strip()) for e in b.iter()]
            for b in ET.parse(path).getroot().findall("ros2_control")]
same = blocks("/evidence/stock.real.urdf") == blocks("/evidence/assembly.real.urdf")
print(f"MOVEIT_{'PASS' if same else 'FAIL'} real rover drivetrain wiring is identical with the arm attached")
EOF
    check_urdf /evidence/assembly.real.urdf >/dev/null && echo "MOVEIT_PASS assembled model passes the URDF parser" \
        || { echo "MOVEIT_FAIL assembled model passes the URDF parser"; STATUS=1; }
    ros2 launch chassis_bringup rover.launch.py mode:=sim joy_control:=false arm_camera:=false >/evidence/rover.log 2>&1 &
    sleep 22
    timeout 70 python /toolkit/tests/ros/arm_modes_check.py hold || STATUS=1
    ros2 launch billee_arm_moveit_config moveit.launch.py >/evidence/moveit.log 2>&1 &
    timeout 230 python /toolkit/tests/ros/arm_moveit_check.py || STATUS=1
    stop_all
    return "$STATUS"
}

standalone() {
    mkdir -p /tmp/ws_arm/src
    cp -r /cases/standalone/billee_arm_moveit_config /tmp/ws_arm/src/
    (cd /tmp/ws_arm && colcon build --packages-select billee_arm_moveit_config >/evidence/build_arm.log 2>&1) \
        && echo "MOVEIT_PASS standalone package builds" || { echo "MOVEIT_FAIL standalone package builds"; tail -15 /evidence/build_arm.log; return 1; }
    . /tmp/ws_arm/install/local_setup.bash
    ros2 launch billee_arm_moveit_config moveit.launch.py >/evidence/standalone.log 2>&1 &
    timeout 110 python /toolkit/tests/ros/arm_moveit_check.py --standalone
    result=$?
    stop_all
    return "$result"
}

( assembly ) || STATUS=1
( standalone ) || STATUS=1
timeout 70 python /toolkit/tests/ros/arm_modes_check.py pxx || STATUS=1
stop_all
trap - EXIT
exit "$STATUS"
