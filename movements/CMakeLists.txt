cmake_minimum_required(VERSION 2.8.3)
project(movements)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  angles
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS angles
  DEPENDS Eigen
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_CODE_BASE
  "src/*.cpp"
)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)
