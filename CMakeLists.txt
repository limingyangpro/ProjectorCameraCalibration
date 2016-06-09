cmake_minimum_required ( VERSION 2.8)
project( ProcamCalibration )

MESSAGE(STATUS “OSG_ROOT = $ENV{OSG_ROOT}”)
MESSAGE(STATUS “OSG_LIBRARY_PATH = $ENV{OSG_LIBRARY_PATH}”)

set(SRCEXT "cpp")

find_package ( OpenCV REQUIRED )
find_package ( PCL 1.7 )
find_package( osg )
find_package( osgDB )
find_package( osgGA )
find_package( osgViewer )
find_package( osgText )

set(EXECUTABLE_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

macro( config_project PROJNAME LIBNAME)
	include_directories( ${${LIBNAME}_INCLUDE_DIR} )
	target_link_libraries( ${PROJNAME} ${${LIBNAME}_LIBRARY} )
endmacro()

if (PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif (PCL_FOUND)

#For OSG
include_directories( ${OSG_INCLUDE_DIR} )
include_directories( ${OSGDB_INCLUDE_DIR} )
include_directories( ${OSGVIEWER_INCLUDE_DIR} )

#set link directories for lgc
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lgc/src)

add_subdirectory( detectionlib )
add_subdirectory( pointpatterngen )
add_subdirectory( caliblib )
add_subdirectory( auglib )

add_executable(procamcalib procamcalib.cpp)

target_link_libraries(
    procamcalib
    caliblib
    lgc
    detectionlib
    Xinerama
    X11
    ${OpenCV_LIBS}
    ${PCL_IO_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${OSG_LIBRARY}
    ${OSGTEXT_LIBRARY}
    ${OSGDB_LIBRARY}
    ${OSGVIEWER_LIBRARY}
    lgc
)

add_executable(augmentation augmentation.cpp)

target_link_libraries(
    augmentation
    auglib
    lgc
    detectionlib
    Xinerama
    X11
    ${OpenCV_LIBS}
    ${PCL_IO_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${OSG_LIBRARY}
    ${OSGTEXT_LIBRARY}
    ${OSGDB_LIBRARY}
    ${OSGGA_LIBRARY} 
    ${OSGVIEWER_LIBRARY}
    lgc
)


add_executable(analyseresult analyseresult.cpp)

target_link_libraries(
    analyseresult
    ${OpenCV_LIBS}
)

add_executable(summarize summarize.cpp)

target_link_libraries(
    summarize
    ${OpenCV_LIBS}
)
