cmake_minimum_required(VERSION 2.8.3)
project(asr_recognizer_prediction_psm)

add_definitions(-Wall -std=gnu++0x -ggdb)


find_package(cmake_modules)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  asr_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  roscpp
  asr_psm
  message_generation
  asr_rapidxml
  asr_lib_ism
)

find_package(Boost REQUIRED
  iostreams
)

find_package(Eigen3 REQUIRED)

include_directories(
  include 
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  recognizer_prediction_psm.srv
  psm_node.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  asr_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS tf
		  roscpp
		  asr_msgs
		  std_msgs
		  std_srvs
		  sensor_msgs
		  geometry_msgs
		  asr_psm
		  message_runtime
		  asr_rapidxml
  #DEPENDS Eigen3 Boost
  DEPENDS Boost
)

add_executable(${PROJECT_NAME} src/recognizer_prediction_psm_server.cpp
				src/asrVisualizer.cpp
				src/asrPosePredictionEngine.cpp
			    )

add_dependencies(
	${PROJECT_NAME}
	${PROJECT_NAME}_generate_messages_cpp
	asr_msgs_generate_messages_cpp
)
			    
add_executable(${PROJECT_NAME}_client src/recognizer_prediction_psm_client.cpp)

add_executable(psm_node src/psm_node_server.cpp)

add_dependencies(
	psm_node
	${PROJECT_NAME}_generate_messages_cpp
	asr_msgs_generate_messages_cpp
)
			    
target_link_libraries(
  ${PROJECT_NAME}
  ${Eigen3_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(
  ${PROJECT_NAME}_client
  ${Eigen3_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(
  psm_node
  ${Eigen3_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)


