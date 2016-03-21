cmake_minimum_required(VERSION 2.8)
project(mmf_writer)

find_package(catkin REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
  message_generation
  cv_bridge
  dynamic_reconfigure
  image_geometry
  image_transport
  nodelet
  roscpp
  std_msgs
  sensor_msgs)
find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
  add_definitions("-DOPENCV3=1")
endif()
find_package(Boost REQUIRED COMPONENTS thread)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/ProcessImage.cfg)

add_message_files(
  FILES
  Ellipse.msg
  Blob.msg
  Blobs.msg
  RecordingStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS image_geometry roscpp sensor_msgs message_runtime image_transport
  DEPENDS OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_library(${PROJECT_NAME}
  src/nodelets/process_image.cpp
  src/nodelets/mmfWriter_nodelet.cpp
  src/nodelets/mmfWriter.cpp
  src/libmmf_writer/advertisement_checker.cpp
  src/libmmf_writer/StaticBackgroundCompressor.cpp
  src/libmmf_writer/BackgroundRemovedImage.cpp
  src/libmmf_writer/IplImageLoaderFixedWidth.cpp
  src/libmmf_writer/BackgroundRemovedImageLoader.cpp
  src/libmmf_writer/ImageMetaDataLoader.cpp
  src/libmmf_writer/NameValueMetaData.cpp
  src/libmmf_writer/CompositeImageMetaData.cpp
  src/libmmf_writer/LinearStackCompressor.cpp
  src/libmmf_writer/Timer.cpp
  src/libmmf_writer/StackReader.cpp
  src/libmmf_writer/ExtraDataWriter.cpp
  src/libmmf_writer/StaticBackgroundCompressorLoader.cpp
  src/libmmf_writer/ThreadedStaticBackgroundCompressor.cpp
  src/libmmf_writer/ThreadedLinearStackCompressor.cpp
  
)

# make sure configure headers are built before any node using them
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# add the publisher example
#add_executable(my_publisher2 src/nodes/my_publisher.cpp)
#target_link_libraries(my_publisher2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# add the publisher example
add_executable(my_publisher2 src/nodes/my_publisher.cpp)
target_link_libraries(my_publisher2 ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Standalone node
add_executable(mmf_writer_exe src/nodes/mmf_writer.cpp)
target_link_libraries(mmf_writer_exe ${PROJECT_NAME}  ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
SET_TARGET_PROPERTIES(mmf_writer_exe PROPERTIES OUTPUT_NAME mmf_writer)
install(TARGETS mmf_writer_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#add_executable(my_publisher_exe src/nodes/my_publisher.cpp)
#target_link_libraries(my_publisher_exe ${PROJECT_NAME}  ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
#SET_TARGET_PROPERTIES(my_publisher_exe PROPERTIES OUTPUT_NAME my_publisher)
#install(TARGETS my_publisher_exe
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


# install the launch file
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
