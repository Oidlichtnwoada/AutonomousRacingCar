# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(f110_simulator_CONFIG_INCLUDED)
  return()
endif()
set(f110_simulator_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(f110_simulator_SOURCE_PREFIX /media/sf_OneDrive/TU/2020S_Semester/Autonomous_Racing_Cars_VU/AutonomousRacingCar/src/f110_ros/f110_simulator)
  set(f110_simulator_DEVEL_PREFIX /media/sf_OneDrive/TU/2020S_Semester/Autonomous_Racing_Cars_VU/AutonomousRacingCar/devel)
  set(f110_simulator_INSTALL_PREFIX "")
  set(f110_simulator_PREFIX ${f110_simulator_DEVEL_PREFIX})
else()
  set(f110_simulator_SOURCE_PREFIX "")
  set(f110_simulator_DEVEL_PREFIX "")
  set(f110_simulator_INSTALL_PREFIX /media/sf_OneDrive/TU/2020S_Semester/Autonomous_Racing_Cars_VU/AutonomousRacingCar/install)
  set(f110_simulator_PREFIX ${f110_simulator_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'f110_simulator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(f110_simulator_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/media/sf_OneDrive/TU/2020S_Semester/Autonomous_Racing_Cars_VU/AutonomousRacingCar/src/f110_ros/f110_simulator/include " STREQUAL " ")
  set(f110_simulator_INCLUDE_DIRS "")
  set(_include_dirs "/media/sf_OneDrive/TU/2020S_Semester/Autonomous_Racing_Cars_VU/AutonomousRacingCar/src/f110_ros/f110_simulator/include")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Hongrui Zheng <billyzheng.bz@gmail.com>, Joe Auckley <jauckley@seas.upenn.edu>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${f110_simulator_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'f110_simulator' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'f110_simulator' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/media/sf_OneDrive/TU/2020S_Semester/Autonomous_Racing_Cars_VU/AutonomousRacingCar/src/f110_ros/f110_simulator/${idir}'.  ${_report}")
    endif()
    _list_append_unique(f110_simulator_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "f110_simulator")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND f110_simulator_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND f110_simulator_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT f110_simulator_NUM_DUMMY_TARGETS)
      set(f110_simulator_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::f110_simulator::wrapped-linker-option${f110_simulator_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR f110_simulator_NUM_DUMMY_TARGETS "${f110_simulator_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::f110_simulator::wrapped-linker-option${f110_simulator_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND f110_simulator_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND f110_simulator_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND f110_simulator_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /media/sf_OneDrive/TU/2020S_Semester/Autonomous_Racing_Cars_VU/AutonomousRacingCar/devel/lib;/media/sf_OneDrive/TU/2020S_Semester/Autonomous_Racing_Cars_VU/AutonomousRacingCar/devel/lib;/media/sf_OneDrive/TU/2020S_Semester/Autonomous_Racing_Cars_VU/exercises/ROS/brantner_ws/devel/lib;/opt/ros/melodic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(f110_simulator_LIBRARY_DIRS ${lib_path})
      list(APPEND f110_simulator_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'f110_simulator'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND f110_simulator_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(f110_simulator_EXPORTED_TARGETS "f110_simulator_generate_messages_cpp;f110_simulator_generate_messages_eus;f110_simulator_generate_messages_lisp;f110_simulator_generate_messages_nodejs;f110_simulator_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${f110_simulator_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "message_runtime")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 f110_simulator_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${f110_simulator_dep}_FOUND)
      find_package(${f110_simulator_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${f110_simulator_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(f110_simulator_INCLUDE_DIRS ${${f110_simulator_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(f110_simulator_LIBRARIES ${f110_simulator_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${f110_simulator_dep}_LIBRARIES})
  _list_append_deduplicate(f110_simulator_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(f110_simulator_LIBRARIES ${f110_simulator_LIBRARIES})

  _list_append_unique(f110_simulator_LIBRARY_DIRS ${${f110_simulator_dep}_LIBRARY_DIRS})
  list(APPEND f110_simulator_EXPORTED_TARGETS ${${f110_simulator_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "f110_simulator-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${f110_simulator_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
