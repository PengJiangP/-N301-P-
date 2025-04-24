cmake_minimum_required(VERSION 3.5)
project(Example_demo)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
include_directories(Include)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(ARCHIVE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/lib)
SET(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/lib)

IF(CMAKE_SYSTEM_NAME MATCHES "Windows")
	add_definitions(-DWIN64)
ELSEIF(CMAKE_SYSTEM_NAME MATCHES "Linux")
	add_definitions(-DLINUX)
ENDIF()

#find all cpp files under Include directory
file(GLOB SRC_NAMES Include/*.cpp)

message("*********: ",${SRC_NAMES})
add_executable(demo demo/main.cpp)
target_link_libraries(demo -pthread)

add_library(LidarResolutionLibraryDll SHARED ${SRC_NAMES})

add_library(LidarResolutionLibraryLib STATIC ${SRC_NAMES})

find_package(PCL QUIET)
if(PCL_FOUND)
	message(=============================================================)
	message("-- PCL Found. PCL Support is turned On.")
	message(=============================================================)
	
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
	
	add_executable(demo_viewer demo/main_PCL.cpp)
	target_link_libraries(demo_viewer -pthread ${PCL_LIBRARIES})


	#find_package(PCAP REQUIRED)
	# Include Header File Directories
	include_directories(./demo/PcapInclude)

	if(CMAKE_CL_64)
		set(CURRENT_PLATFORM "x64")
		message(STATUS "Current Platform is ${CURRENT_PLATFORM}")
		link_directories(./demo/PcapLib/x64)
	else(CMAKE_CL_64)
		set(CURRENT_PLATFORM "x86")
		message(STATUS "Current Platform is ${CURRENT_PLATFORM}")
		link_directories(./demo/PcapLib)
	endif(CMAKE_CL_64)

	add_executable(demo_viewer_Pcap demo/main_PCL_Pcap.cpp)
	target_link_libraries(demo_viewer_Pcap -pthread ${PCL_LIBRARIES} -lpcap)
	
else(PCL_FOUND)
	message("=================================================")
	message(STATUS "-- PCL Not Found. PCL Suport is turned off")
	message("=================================================")
endif(PCL_FOUND)


