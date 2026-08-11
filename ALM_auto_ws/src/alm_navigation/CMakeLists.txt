cmake_minimum_required(VERSION 3.8)
project(alm_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# behavior_trees: SmoothPath 를 끼운 커스텀 BT. navigation.launch.py 가
# 설치 경로를 RewrittenYaml 로 bt_navigator 에 주입하므로 반드시 설치돼야 한다.
install(
  DIRECTORY behavior_trees config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/pcd2pgm.py
    scripts/map_publisher.py
    scripts/pcd_replay.py
    scripts/map_manager.py
    scripts/migrate_maps.py
    scripts/prior_cloud_publisher.py
    scripts/nav2_kinematic_check.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
