# CMake generated Testfile for 
# Source directory: /home/amap/livox_catkin_ws/src/loam_velodyne
# Build directory: /home/amap/livox_catkin_ws/src/build/loam_velodyne
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_loam_velodyne_rostest_test_loam.test "/home/amap/livox_catkin_ws/src/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/amap/livox_catkin_ws/src/build/test_results/loam_velodyne/rostest-test_loam.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/amap/livox_catkin_ws/src/loam_velodyne --package=loam_velodyne --results-filename test_loam.xml --results-base-dir \"/home/amap/livox_catkin_ws/src/build/test_results\" /home/amap/livox_catkin_ws/src/build/loam_velodyne/test/loam.test ")
set_tests_properties(_ctest_loam_velodyne_rostest_test_loam.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/amap/livox_catkin_ws/src/loam_velodyne/CMakeLists.txt;63;add_rostest;/home/amap/livox_catkin_ws/src/loam_velodyne/CMakeLists.txt;0;")
subdirs("src/lib")
