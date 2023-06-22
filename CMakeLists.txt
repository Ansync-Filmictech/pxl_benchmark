cmake_minimum_required(VERSION 3.0)
project( Benchmark)
find_package( OpenCV REQUIRED )

add_definitions(-DPIXELINK_LINUX)
set(PIXELINK_HOME /opt)
set(PIXELINK_DIR ${PIXELINK_HOME}/PixeLINKSdk)
set(PIXELINK_SOURCES ${PIXELINK_DIR}/lib/libPxLApi.so)

set(OIIO_HOME  /opt/oiio/dist)
set(OpenImageIO_INCLUDE_DIR  ${OIIO_HOME}/include)
set(OpenImageIO_LIBRARY ${OIIO_HOME}/lib/libOpenImageIO.so)
set(OpenImageIO_Util_LIBRARY ${OIIO_HOME}/lib/libOpenImageIO_Util.so)

add_subdirectory(src)

include_directories( 
	src/
	${OpenCV_INCLUDE_DIRS} 
    ${OpenImageIO_INCLUDE_DIR}
    ${PIXELINK_HOME}/PixeLINKSdk/include
)

add_executable( Benchmark src/benchmark.cpp ${source})

target_link_libraries( Benchmark ${OpenCV_LIBS} 
    ${OpenImageIO_LIBRARY} ${OpenImageIO_Util_LIBRARY}
    ${PIXELINK_SOURCES}
)


