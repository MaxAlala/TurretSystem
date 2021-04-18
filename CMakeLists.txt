project( turretSystem C CXX)
	
cmake_minimum_required(VERSION 3.0)

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Build type automatically set to 'Release'")
    set(CMAKE_BUILD_TYPE "Release" )
else(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Build type: '${CMAKE_BUILD_TYPE}'") 
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        message(STATUS "WARNING: you will get poor performance!")
    endif(CMAKE_BUILD_TYPE STREQUAL "Debug")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set (CMAKE_CXX_STANDARD 14)
option(USE_SSE "Build with SSE3+ support" ON)
option(USE_AVX "Build with AVX/AVX2 support" OFF)
option(USE_NEON "Build with NEON support" OFF)
option(USE_CAFFE_TRIPLET "Build with Caffe Triplet CNN support" OFF)
### ZED
option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()


#################################
SET(THREADS_HAVE_PTHREAD_ARG 1)
FIND_PACKAGE(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL libserial)




set(CMAKE_CURRENT_SOURCE_DIR ~/git_ws/TurretSystem)
# This is to be complinat with OpenCV >= 3.3.1
set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -DOPENCV_TRAITS_ENABLE_DEPRECATED")
  
  set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -Wall -Wpedantic -Wno-narrowing -Wno-deprecated -std=c++14 -march=nocona")
  find_package(Eigen3 REQUIRED)
#find_package(OpenCV REQUIRED PATHS /home/northlight/libs/opencv4.2/install NO_DEFAULT_PATH)
#find_package(OpenCV REQUIRED PATHS "/usr/opencv-2.4.wwww10")


### adds OpenCV include + cpp files
set(OpenCV_INCLUDE_DIRS /home/sol/d2co/libsForD2co/opencv/install/include
)
set(OpenCV_LIBS  opencv_calib3d;opencv_core;opencv_dnn;opencv_features2d;opencv_flann;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_shape;opencv_stitching;opencv_superres;opencv_video;opencv_videoio;opencv_videostab;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_datasets;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hdf;opencv_img_hash;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_reg;opencv_rgbd;opencv_saliency;opencv_stereo;opencv_structured_light;opencv_surface_matching;opencv_text;opencv_tracking;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto
)
SET("OpenCV_DIR" "/home/sol/d2co/libsForD2co/opencv")

# specifies a directory where linker will find library
#Note that this command is rarely necessary. Library locations returned by find_package() and find_library() are absolute paths. Pass these absolute library file paths directly to the target_link_libraries() command. CMake will ensure the linker finds them.
link_directories(/home/sol/d2co/libsForD2co/opencv/install/lib)
#aux_source_directory( src OPENCV_PROJECTS_SRC )
###end

### adds cMake folder
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

###end

### adds eigen3 folder
if(NOT DEFINED EIGEN3_INCLUDE_DIR)

#This command is used to find a directory containing the named file. A cache entry named by <VAR> is created to store the result of this command. If the file in a directory is found the result is stored in the variable and the search will not be repeated unless the variable is cleared. If nothing is found, the result will be <VAR>-NOTFOUND. The REQUIRED option stops processing with an error message if nothing is found, otherwise the search will be attempted again the next time find_path is invoked with the same variable.
  find_path(EIGEN3_INCLUDE_DIR eigen3)
endif(NOT DEFINED EIGEN3_INCLUDE_DIR)
###end
 


###add TurretSystem
set(
TurretSystemSubClasses 
src/MovementDetector.cpp
src/motionDetector.cpp 
src/PixelToMotorStepsConverter.cpp
src/StepperMotorController.cpp
src/TurretSystem.cpp
src/Serial_Rs232.cpp
src/Eye.cpp
src/InverseForwardKinematicsModel.cpp

)

add_library(TurretSystemSubClasses STATIC ${TurretSystemSubClasses})

target_link_libraries( TurretSystemSubClasses PUBLIC
                       ${Boost_LIBRARIES}
                       ${OpenCV_LIBS}
                       ${SERIAL_LDFLAGS} 
                       ${CMAKE_THREAD_LIBS_INIT}
)
#set(SERIAL_LDFLAGS -L/usr/local/lib)
set_target_properties(TurretSystemSubClasses PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set_target_properties(TurretSystemSubClasses PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
###end

### adds include folders
#Add the given directories to those the compiler uses to search for include files. Relative paths are interpreted as relative to the current source directory.
include_directories( ./include
                      /usr/include
                      /usr/local/include
                      ${OpenCV_INCLUDE_DIRS}
                      ${Boost_INCLUDE_DIRS}
                      ${EIGEN3_INCLUDE_DIR}
	              ${PATH_INCLUDE_OPENCV}
		      ${SERIAL_INCLUDE_DIRS}
                       /include
)
###end

###print messages
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")
message(STATUS "OpenCV_DIRS = ${OpenCV_DIR}")
message(STATUS "OPENCV_PROJECTS_SRC = ${OPENCV_PROJECTS_SRC}")
message(STATUS "SERIAL_LDFLAGS === ${SERIAL_LDFLAGS}" )
message(STATUS "HELLLLLLLLOOOOO!")
message(STATUS "SERIAL_INCLUDE_DIRS == ${SERIAL_INCLUDE_DIRS}")
###end

#Targets can have properties that affect how they are built.
#
#set_target_properties(target1 target2 ...
#                      PROPERTIES prop1 value1
#                      prop2 value2 ...)
#Sets properties on targets. The syntax for the command is to list all the targets you want to change, and then provide the values you want to set next. You can use any prop value pair you want and extract it later with the get_property() or get_target_property() command.
#set(SERIAL_LDFLAGS -L/usr/local/lib)

add_executable(turretSystem src/main.cpp)
set_target_properties(turretSystem PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set_target_properties(turretSystem PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set_target_properties( turretSystem PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set_target_properties( turretSystem PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries( turretSystem
           ${Boost_LIBRARIES}
           ${OpenCV_LIBS}
	   ${SERIAL_LDFLAGS} 
           ${CMAKE_THREAD_LIBS_INIT}
           TurretSystemSubClasses
)
#
#add_executable(dense_optical_flow4 apps/dense_optical_flow4.cpp)
#set_target_properties( dense_optical_flow4 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
#set_target_properties( dense_optical_flow4 PROPERTIES LINKER_LANGUAGE CXX)
#target_link_libraries( dense_optical_flow4
#           ${OpenCV_LIBS}
#	   ${SERIAL_LDFLAGS} 
#	   ${CMAKE_THREAD_LIBS_INIT}
#)

add_executable(script1 apps/script1.cpp)
set_target_properties( script1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set_target_properties( script1 PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries( script1
           ${OpenCV_LIBS}
	   ${SERIAL_LDFLAGS} 
	   ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(script2 apps/script2.cpp)
set_target_properties( script2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set_target_properties( script2 PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries( script2
           ${OpenCV_LIBS}
	   ${SERIAL_LDFLAGS} 
	   ${CMAKE_THREAD_LIBS_INIT}
)


add_executable(script3 apps/script3.cpp)
set_target_properties( script3 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set_target_properties( script3 PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries( script3
           ${OpenCV_LIBS}
	   ${SERIAL_LDFLAGS} 
	   ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(script4 apps/script4.cpp)
set_target_properties( script4 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set_target_properties( script4 PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries( script4
           ${OpenCV_LIBS}
	   ${SERIAL_LDFLAGS} 
	   ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(script5 apps/script5.cpp)
set_target_properties( script5 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set_target_properties( script5 PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries( script5
           ${OpenCV_LIBS}
	   ${SERIAL_LDFLAGS} 
	   ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(script6 apps/script6.cpp)
set_target_properties( script6 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set_target_properties( script6 PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries( script6
           ${OpenCV_LIBS}
	   ${SERIAL_LDFLAGS} 
	   ${CMAKE_THREAD_LIBS_INIT}
)


add_executable(calibrate src/calib_intrinsic.cpp src/popt_pp.h)
set_target_properties(calibrate PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set_target_properties( calibrate PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(calibrate ${OpenCV_LIBS} "-lpopt")