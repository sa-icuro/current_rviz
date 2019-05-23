execute_process(COMMAND "/home/centaur/Desktop/ubi_tools/build/rviz_tools_py/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/centaur/Desktop/ubi_tools/build/rviz_tools_py/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
