cmake_minimum_required(VERSION 3.4)

project(MyFilter)

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(Qt5 COMPONENTS Widgets REQUIRED)
include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_definitions(${Qt5Widgets_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES})

find_package(SlicerExecutionModel REQUIRED)
include(${SlicerExecutionModel_USE_FILE})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

#add_executable(MyFilterApp MyFilterApp.cxx)
#target_link_libraries(MyFilterApp ${ITK_LIBRARIES})


SEMMacroBuildCLI(
	NAME MyFilterApp
	EXECUTABLE_ONLY
	TARGET_LIBRARIES ${ITK_LIBRARIES} ${VTK_LIBRARIES} ${QT_LIBRARIES}
)

if(BUILD_TESTING)
	add_subdirectory(Testing)
endif()


