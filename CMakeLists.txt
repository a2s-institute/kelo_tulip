cmake_minimum_required(VERSION 3.5.1)
project(kelo_tulip)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find_package(catkin REQUIRED COMPONENTS
# 	roscpp
# 	geometry_msgs
# 	nav_msgs
# 	sensor_msgs
# 	std_msgs
# 	tf
# )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

#set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS thread)

if(WIN32)
	set(INCLUDE_DIRS_KELO
		include
		include/kelo_tulip/soem
		include/kelo_tulip/soem/osal
		include/kelo_tulip/soem/osal/win32
		include/kelo_tulip/soem/oshw/win32
		${WINPCAP_INCLUDE_DIRS}
	)
else(WIN32)
	set(INCLUDE_DIRS_KELO
		include
		include/kelo_tulip/soem
		include/kelo_tulip/soem/osal
		include/kelo_tulip/soem/osal/linux
		include/kelo_tulip/soem/oshw/linux
	)
endif(WIN32)

include_directories(
	${INCLUDE_DIRS_KELO}
	# ${catkin_INCLUDE_DIRS}
)

# Optionally apply cap_net_raw+ep permissions to binary. Requires root previleges.
# Use sudo as default. Still, allow to be configured as environment veriable.
OPTION(USE_SETCAP "Set permissions to access ethernet interface without sudo" ON)
IF("$ENV{SUDO_COMMAND}" STREQUAL "")
	SET(SUDO_COMMAND sudo)				# default = sudo
ELSE("$ENV{SUDO_COMMAND}" STREQUAL "")
	SET(SUDO_COMMAND $ENV{SUDO_COMMAND})
ENDIF("$ENV{SUDO_COMMAND}" STREQUAL "")

add_subdirectory(src/soem)

add_library(tulip_velocity_controller
	src/Utils.cpp
	src/VelocityPlatformController.cpp
)

target_link_libraries(tulip_velocity_controller
	# ${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${rclcpp_LIBRARIES}
)

add_executable(platform_driver
	src/EtherCATModule.cpp
	src/PlatformDriver.cpp
	src/PlatformDriverROS.cpp
	src/modules/RobileMasterBattery.cpp
)

target_link_libraries(platform_driver
	tulip_velocity_controller
	# ${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	soem
	pthread
	${rclcpp_LIBRARIES}
)

IF(USE_SETCAP)
  add_custom_command(TARGET platform_driver POST_BUILD
  	COMMAND ${SUDO_COMMAND} setcap cap_net_raw+ep $<TARGET_FILE:platform_driver>
  )
endif(USE_SETCAP)

install(TARGETS platform_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config/
	DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(
	include
	${INCLUDE_DIRS_KELO}
)

ament_export_libraries(
	tulip_velocity_controller
)

ament_package(
	# INCLUDE_DIRS ${INCLUDE_DIRS} 
	# LIBRARIES tulip_velocity_controller
)
