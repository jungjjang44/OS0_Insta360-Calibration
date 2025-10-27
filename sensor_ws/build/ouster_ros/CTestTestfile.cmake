# CMake generated Testfile for 
# Source directory: /home/krri/OS0_Insta360-Calibration/sensor_ws/src/ouster-ros
# Build directory: /home/krri/OS0_Insta360-Calibration/sensor_ws/build/ouster_ros
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ouster_ros_test "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/krri/OS0_Insta360-Calibration/sensor_ws/build/ouster_ros/test_results/ouster_ros/ouster_ros_test.gtest.xml" "--package-name" "ouster_ros" "--output-file" "/home/krri/OS0_Insta360-Calibration/sensor_ws/build/ouster_ros/ament_cmake_gtest/ouster_ros_test.txt" "--command" "/home/krri/OS0_Insta360-Calibration/sensor_ws/build/ouster_ros/ouster_ros_test" "--gtest_output=xml:/home/krri/OS0_Insta360-Calibration/sensor_ws/build/ouster_ros/test_results/ouster_ros/ouster_ros_test.gtest.xml")
set_tests_properties(ouster_ros_test PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/krri/OS0_Insta360-Calibration/sensor_ws/build/ouster_ros/ouster_ros_test" TIMEOUT "60" WORKING_DIRECTORY "/home/krri/OS0_Insta360-Calibration/sensor_ws/build/ouster_ros" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/krri/OS0_Insta360-Calibration/sensor_ws/src/ouster-ros/CMakeLists.txt;191;ament_add_gtest;/home/krri/OS0_Insta360-Calibration/sensor_ws/src/ouster-ros/CMakeLists.txt;0;")
subdirs("ouster_example")
subdirs("gtest")
