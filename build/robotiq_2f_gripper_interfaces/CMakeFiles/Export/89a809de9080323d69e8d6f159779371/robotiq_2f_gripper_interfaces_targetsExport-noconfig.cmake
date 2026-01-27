#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robotiq_2f_gripper_interfaces::robotiq_2f_gripper_interfaces" for configuration ""
set_property(TARGET robotiq_2f_gripper_interfaces::robotiq_2f_gripper_interfaces APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(robotiq_2f_gripper_interfaces::robotiq_2f_gripper_interfaces PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librobotiq_2f_gripper_interfaces.so"
  IMPORTED_SONAME_NOCONFIG "librobotiq_2f_gripper_interfaces.so"
  )

list(APPEND _cmake_import_check_targets robotiq_2f_gripper_interfaces::robotiq_2f_gripper_interfaces )
list(APPEND _cmake_import_check_files_for_robotiq_2f_gripper_interfaces::robotiq_2f_gripper_interfaces "${_IMPORT_PREFIX}/lib/librobotiq_2f_gripper_interfaces.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
