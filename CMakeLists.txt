cmake_minimum_required(VERSION 3.5)
project(hal_lucidlabs_helios2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic 
  -O3 -mtune=native -march=native
  #-g -ggdb
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io geometry search features)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(pcl_conversions REQUIRED)

message("SDK path:           " $ENV{ARENA_PATH})
message("SDK build version:  " $ENV{ARENA_GCCVER})

include_directories( SYSTEM
  include
  $ENV{ARENA_PATH}/include/Arena
	$ENV{ARENA_PATH}/include/Save
	$ENV{ARENA_PATH}/include/GenTL
	$ENV{ARENA_PATH}/GenICam/library/CPP/include
)

link_directories(
  $ENV{ARENA_PATH}/lib64
  $ENV{ARENA_PATH}/ffmpeg
  $ENV{ARENA_PATH}/GenICam/library/lib/Linux64_x64
)

add_library(HALHelios SHARED src/lucidlabs_helios2.cpp)
target_link_libraries(HALHelios 
  -larena
  -lsave
  -lgentl
  -lGCBase_gcc$ENV{ARENA_GCCVER}
  -lGenApi_gcc$ENV{ARENA_GCCVER}
  -lLog_gcc$ENV{ARENA_GCCVER}
  -llog4cpp_gcc$ENV{ARENA_GCCVER}
  -lMathParser_gcc$ENV{ARENA_GCCVER}
  -lNodeMapData_gcc$ENV{ARENA_GCCVER}
  -lXmlParser_gcc$ENV{ARENA_GCCVER}
  -lavcodec
  -lavformat
  -lavutil
  -lswresample)
ament_target_dependencies(HALHelios rclcpp rcutils std_msgs sensor_msgs geometry_msgs pcl_conversions)

rclcpp_components_register_node(
  HALHelios
  PLUGIN "hal::LucidlabsHelios2"
  EXECUTABLE HALHeliosExecutable)

install(TARGETS HALHelios
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
