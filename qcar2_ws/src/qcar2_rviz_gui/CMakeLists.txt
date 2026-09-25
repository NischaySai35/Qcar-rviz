cmake_minimum_required(VERSION 3.8)
project(qcar2_rviz_gui)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS
  scripts/qcar2_drive_gui.py
  scripts/qcar2_pose_marker.py
  scripts/qcar2_model_joint_state.py
  scripts/image_preview_throttle.py
  scripts/wheel_imu_odometry.py
  scripts/qcar2_nav_visualizer.py
  scripts/qcar2_goal_reset.py
  scripts/qcar2_goal_heading.py
  scripts/qcar2_speed_limit_gui.py
  scripts/qcar2_announcer.py
  scripts/qcar2_web_gui.py
  scripts/qcar2_object_mapper.py
  scripts/qcar2_object_nav.py
  scripts/qcar2_voice_command.py
  scripts/qcar2_explorer.py
  scripts/qcar2_assistant.py
  DESTINATION lib/${PROJECT_NAME})

# Browser console served by qcar2_web_gui.py (symlink-installed, so edits to
# index.html are live on the next page reload without a rebuild).
install(DIRECTORY web DESTINATION share/${PROJECT_NAME})

ament_package()
