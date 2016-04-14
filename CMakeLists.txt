##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(LaserScannerApplication)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp pcl_conversions pcl_ros cv_bridge)
include_directories(${catkin_INCLUDE_DIRS})

#SET(GCC_COMPILE_OPTIONS "-std=c++11")
#SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COMPILE_OPTIONS}" )

set(ENV{OpenCV_DIR} /usr/local/share/OpenCV)

set(BOOST_ROOT /home/minions/Workspaces/boost1.60/)

#set(Boost_USE_STATIC_LIBS OFF)
#set(Boost_USE_MULTITHREADED ON)
#set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.60.0 COMPONENTS atomic )

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_DIR} ${Boost_INCLUDE_DIRS})

find_package(VTK REQUIRED)
FIND_PACKAGE(VTK)
INCLUDE(${VTK_USE_FILE})
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/LaserScannerApplication/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(LaserScannerApplication ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(LaserScannerApplication ${QT_LIBRARIES} ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${OPENCV_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS LaserScannerApplication RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
