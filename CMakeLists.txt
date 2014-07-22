cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(${PROJECT_NAME} 
    src/Scheduler.cpp 
    src/loop.cpp 
    src/servTest.cpp
    src/RobotControl.cpp 
    src/HuboMotor.cpp 
    src/MotorBoard.cpp 
    src/IMUBoard.cpp 
    src/FTSensorBoard.cpp 
    src/PowerControlBoard.cpp 
    src/HuboState.cpp 
    src/pugixml.cpp 
    src/TrajHandler.cpp
    src/Trajectory.cpp 
    src/WSVFile.cpp 
    src/CommandChannel.cpp 
    src/ReferenceChannel.cpp 
    src/StateChannel.cpp 
    src/SimChannels.cpp
    src/ArmWristXYZ.cpp
    src/Interpolable.cpp
    src/Interpolation.cpp
    src/LowerBodyLeg.cpp
    src/MetaJointController.cpp
    src/MetaJoint.cpp
    src/Names.cpp
    src/NeckRollPitch.cpp
    src/RobotComponent.cpp
    src/ArmMetaJoint.cpp
    src/BalanceController.cpp
    include/Singleton.h)
target_link_libraries(${PROJECT_NAME} ach)
#target_link_libraries(example ${PROJECT_NAME})
