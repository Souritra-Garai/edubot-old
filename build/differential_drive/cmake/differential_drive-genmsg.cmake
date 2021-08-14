# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "differential_drive: 1 messages, 1 services")

set(MSG_I_FLAGS "-Idifferential_drive:/home/souritra/edubot/src/differential_drive/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(differential_drive_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/souritra/edubot/src/differential_drive/msg/WheelAngularVelocityPair.msg" NAME_WE)
add_custom_target(_differential_drive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "differential_drive" "/home/souritra/edubot/src/differential_drive/msg/WheelAngularVelocityPair.msg" ""
)

get_filename_component(_filename "/home/souritra/edubot/src/differential_drive/srv/SetFloatParam.srv" NAME_WE)
add_custom_target(_differential_drive_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "differential_drive" "/home/souritra/edubot/src/differential_drive/srv/SetFloatParam.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(differential_drive
  "/home/souritra/edubot/src/differential_drive/msg/WheelAngularVelocityPair.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/differential_drive
)

### Generating Services
_generate_srv_cpp(differential_drive
  "/home/souritra/edubot/src/differential_drive/srv/SetFloatParam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/differential_drive
)

### Generating Module File
_generate_module_cpp(differential_drive
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/differential_drive
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(differential_drive_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(differential_drive_generate_messages differential_drive_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/souritra/edubot/src/differential_drive/msg/WheelAngularVelocityPair.msg" NAME_WE)
add_dependencies(differential_drive_generate_messages_cpp _differential_drive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/souritra/edubot/src/differential_drive/srv/SetFloatParam.srv" NAME_WE)
add_dependencies(differential_drive_generate_messages_cpp _differential_drive_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(differential_drive_gencpp)
add_dependencies(differential_drive_gencpp differential_drive_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS differential_drive_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(differential_drive
  "/home/souritra/edubot/src/differential_drive/msg/WheelAngularVelocityPair.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/differential_drive
)

### Generating Services
_generate_srv_eus(differential_drive
  "/home/souritra/edubot/src/differential_drive/srv/SetFloatParam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/differential_drive
)

### Generating Module File
_generate_module_eus(differential_drive
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/differential_drive
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(differential_drive_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(differential_drive_generate_messages differential_drive_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/souritra/edubot/src/differential_drive/msg/WheelAngularVelocityPair.msg" NAME_WE)
add_dependencies(differential_drive_generate_messages_eus _differential_drive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/souritra/edubot/src/differential_drive/srv/SetFloatParam.srv" NAME_WE)
add_dependencies(differential_drive_generate_messages_eus _differential_drive_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(differential_drive_geneus)
add_dependencies(differential_drive_geneus differential_drive_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS differential_drive_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(differential_drive
  "/home/souritra/edubot/src/differential_drive/msg/WheelAngularVelocityPair.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/differential_drive
)

### Generating Services
_generate_srv_lisp(differential_drive
  "/home/souritra/edubot/src/differential_drive/srv/SetFloatParam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/differential_drive
)

### Generating Module File
_generate_module_lisp(differential_drive
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/differential_drive
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(differential_drive_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(differential_drive_generate_messages differential_drive_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/souritra/edubot/src/differential_drive/msg/WheelAngularVelocityPair.msg" NAME_WE)
add_dependencies(differential_drive_generate_messages_lisp _differential_drive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/souritra/edubot/src/differential_drive/srv/SetFloatParam.srv" NAME_WE)
add_dependencies(differential_drive_generate_messages_lisp _differential_drive_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(differential_drive_genlisp)
add_dependencies(differential_drive_genlisp differential_drive_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS differential_drive_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(differential_drive
  "/home/souritra/edubot/src/differential_drive/msg/WheelAngularVelocityPair.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/differential_drive
)

### Generating Services
_generate_srv_nodejs(differential_drive
  "/home/souritra/edubot/src/differential_drive/srv/SetFloatParam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/differential_drive
)

### Generating Module File
_generate_module_nodejs(differential_drive
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/differential_drive
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(differential_drive_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(differential_drive_generate_messages differential_drive_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/souritra/edubot/src/differential_drive/msg/WheelAngularVelocityPair.msg" NAME_WE)
add_dependencies(differential_drive_generate_messages_nodejs _differential_drive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/souritra/edubot/src/differential_drive/srv/SetFloatParam.srv" NAME_WE)
add_dependencies(differential_drive_generate_messages_nodejs _differential_drive_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(differential_drive_gennodejs)
add_dependencies(differential_drive_gennodejs differential_drive_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS differential_drive_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(differential_drive
  "/home/souritra/edubot/src/differential_drive/msg/WheelAngularVelocityPair.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/differential_drive
)

### Generating Services
_generate_srv_py(differential_drive
  "/home/souritra/edubot/src/differential_drive/srv/SetFloatParam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/differential_drive
)

### Generating Module File
_generate_module_py(differential_drive
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/differential_drive
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(differential_drive_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(differential_drive_generate_messages differential_drive_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/souritra/edubot/src/differential_drive/msg/WheelAngularVelocityPair.msg" NAME_WE)
add_dependencies(differential_drive_generate_messages_py _differential_drive_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/souritra/edubot/src/differential_drive/srv/SetFloatParam.srv" NAME_WE)
add_dependencies(differential_drive_generate_messages_py _differential_drive_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(differential_drive_genpy)
add_dependencies(differential_drive_genpy differential_drive_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS differential_drive_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/differential_drive)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/differential_drive
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/differential_drive)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/differential_drive
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/differential_drive)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/differential_drive
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/differential_drive)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/differential_drive
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/differential_drive)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/differential_drive\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/differential_drive
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
