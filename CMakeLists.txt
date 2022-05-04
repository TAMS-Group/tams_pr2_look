cmake_minimum_required(VERSION 2.8.3)
project(tams_pr2_look)

set(MSG_DEPS
	geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	${MSG_DEPS}
)

add_service_files(FILES
	SetTarget.srv
)

add_message_files(FILES
	State.msg
)

generate_messages(DEPENDENCIES
	${MSG_DEPS})

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		${MSG_DEPS}
)
