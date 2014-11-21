cmake_minimum_required(VERSION 2.8.3)
project(rvmaster)

get_filename_component(PARENT_DIR ${CMAKE_SOURCE_DIR} PATH)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Boost REQUIRED COMPONENTS regex program_options)
catkin_package(
  INCLUDE_DIRS include 
  )

include_directories(include
  ${catkin_INCLUDE_DIRS}
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} )

add_executable( ${PROJECT_NAME}_rosmaster 
  src/main.cpp
  src/rvmonitor.cpp 
  src/xmlrpcpp/XmlRpcClient.cpp 
  src/xmlrpcpp/XmlRpcServerConnection.cpp 
  src/xmlrpcpp/XmlRpcServerMethod.cpp 
  src/xmlrpcpp/XmlRpcSocket.cpp 
  src/xmlrpcpp/XmlRpcServer.cpp 
  src/xmlrpcpp/XmlRpcSource.cpp
  src/xmlrpcpp/XmlRpcDispatch.cpp
  src/rv/xmlrpc_manager.cpp 
  src/rv/connection_manager.cpp 
  src/rv/transport_publisher_link.cpp 
  src/rv/transport_subscriber_link.cpp 
  src/rv/subscription.cpp 
  src/rv/publisher_link.cpp 
  src/rv/server_manager.cpp 
  src/rv/master.cpp 
  src/rv/acctrl_manager.cpp 
  src/rv/monitor.cpp
  )

target_link_libraries( ${PROJECT_NAME}_rosmaster 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  )

install( TARGETS ${PROJECT_NAME}_rosmaster
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install( PROGRAMS ${PARENT_DIR}/bin/${PROJECT_NAME}.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install( DIRECTORY ${PARENT_DIR}/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE
  PATTERN ".*.swp" EXCLUDE
  )

# Build live test cases (not using gtest)
foreach( live
    live_sub
    )
  add_executable( ${live} ${CMAKE_SOURCE_DIR}/test/${live}.cpp )
  if(TARGET ${live})
    target_link_libraries(${live} 
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
      )
  endif()
endforeach()
