# robot_arm_ADA526
do ~/.bashrc 
export PYTHONPATH="${PYTHONPATH}:/home/rocotics/robot_arm_ADA526/lib"
alias set_speed='ros2 run robot_arm_pkg set_speed'
alias disable_torque='ros2 run robot_arm_pkg disable_torque'
alias go_home='ros2 run robot_arm_pkg go_home'