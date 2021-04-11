# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "leaf_tracking: 1 messages, 1 services")

set(MSG_I_FLAGS "-Ileaf_tracking:/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(leaf_tracking_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_leaf_tracking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "leaf_tracking" "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/srv/AddTwoInts.srv" ""
)

get_filename_component(_filename "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg/Num.msg" NAME_WE)
add_custom_target(_leaf_tracking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "leaf_tracking" "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg/Num.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(leaf_tracking
  "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leaf_tracking
)

### Generating Services
_generate_srv_cpp(leaf_tracking
  "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leaf_tracking
)

### Generating Module File
_generate_module_cpp(leaf_tracking
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leaf_tracking
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(leaf_tracking_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(leaf_tracking_generate_messages leaf_tracking_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(leaf_tracking_generate_messages_cpp _leaf_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg/Num.msg" NAME_WE)
add_dependencies(leaf_tracking_generate_messages_cpp _leaf_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(leaf_tracking_gencpp)
add_dependencies(leaf_tracking_gencpp leaf_tracking_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leaf_tracking_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(leaf_tracking
  "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/leaf_tracking
)

### Generating Services
_generate_srv_eus(leaf_tracking
  "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/leaf_tracking
)

### Generating Module File
_generate_module_eus(leaf_tracking
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/leaf_tracking
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(leaf_tracking_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(leaf_tracking_generate_messages leaf_tracking_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(leaf_tracking_generate_messages_eus _leaf_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg/Num.msg" NAME_WE)
add_dependencies(leaf_tracking_generate_messages_eus _leaf_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(leaf_tracking_geneus)
add_dependencies(leaf_tracking_geneus leaf_tracking_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leaf_tracking_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(leaf_tracking
  "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leaf_tracking
)

### Generating Services
_generate_srv_lisp(leaf_tracking
  "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leaf_tracking
)

### Generating Module File
_generate_module_lisp(leaf_tracking
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leaf_tracking
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(leaf_tracking_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(leaf_tracking_generate_messages leaf_tracking_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(leaf_tracking_generate_messages_lisp _leaf_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg/Num.msg" NAME_WE)
add_dependencies(leaf_tracking_generate_messages_lisp _leaf_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(leaf_tracking_genlisp)
add_dependencies(leaf_tracking_genlisp leaf_tracking_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leaf_tracking_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(leaf_tracking
  "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/leaf_tracking
)

### Generating Services
_generate_srv_nodejs(leaf_tracking
  "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/leaf_tracking
)

### Generating Module File
_generate_module_nodejs(leaf_tracking
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/leaf_tracking
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(leaf_tracking_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(leaf_tracking_generate_messages leaf_tracking_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(leaf_tracking_generate_messages_nodejs _leaf_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg/Num.msg" NAME_WE)
add_dependencies(leaf_tracking_generate_messages_nodejs _leaf_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(leaf_tracking_gennodejs)
add_dependencies(leaf_tracking_gennodejs leaf_tracking_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leaf_tracking_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(leaf_tracking
  "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leaf_tracking
)

### Generating Services
_generate_srv_py(leaf_tracking
  "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leaf_tracking
)

### Generating Module File
_generate_module_py(leaf_tracking
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leaf_tracking
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(leaf_tracking_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(leaf_tracking_generate_messages leaf_tracking_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(leaf_tracking_generate_messages_py _leaf_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sydney/Desktop/projects/catkin_ws_lt/src/leaf_tracking/msg/Num.msg" NAME_WE)
add_dependencies(leaf_tracking_generate_messages_py _leaf_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(leaf_tracking_genpy)
add_dependencies(leaf_tracking_genpy leaf_tracking_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leaf_tracking_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leaf_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leaf_tracking
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(leaf_tracking_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/leaf_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/leaf_tracking
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(leaf_tracking_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leaf_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leaf_tracking
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(leaf_tracking_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/leaf_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/leaf_tracking
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(leaf_tracking_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leaf_tracking)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leaf_tracking\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leaf_tracking
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(leaf_tracking_generate_messages_py std_msgs_generate_messages_py)
endif()
