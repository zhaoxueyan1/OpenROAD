
# set(CMAKE_CXX_STANDARD 14)

# Please set your environment
# set(TCLLIB "-L ../module/tcl/unix/install-sp/lib -ltcl")
# set(ZLIB_HOME "__YOUR_ZLIB_HOME__/lib")
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g -ggdb") 
include("openroad")
# Below references submodule 
set(OPENSTA_HOME ${CMAKE_CURRENT_SOURCE_DIR}/../../sta)
set(OPENSTA_HDEAR_HOME "${OPENSTA_HOME}/include/")
set(OPENSTA_para_HDEAR_HOME "${OPENSTA_HOME}/parasitics/")

set(dbSTA_HDEAR_HOME "${CMAKE_CURRENT_SOURCE_DIR}/../../dbSta/include/")
message(STATUS "OPENSTA_HDEAR_HOME: ${OPENSTA_HDEAR_HOME}")
message(STATUS "dbSTA_HDEAR_HOME: ${dbSTA_HDEAR_HOME}")
message(STATUS "OPENSTA_HOME: ${OPENSTA_HOME}")
# include_directories(${ZLIB_HOME}/../include)
include_directories(${OPENSTA_HDEAR_HOME}
                    ${dbSTA_HDEAR_HOME}
                    ${OPENSTA_HDEAR_HOME}/sta
                    ${OPENSTA_para_HDEAR_HOME}
                    ${TCL_INCLUDE_PATH}
                    ${OPENSTA_HOME}
                    )

file(GLOB SOURCES "*.cpp")

add_executable(TritonSizer ${SOURCES})

target_link_libraries(TritonSizer 
    ${TCL_LIB}
    utl_lib
    utl
    odb
    OpenSTA
    dbSta
    grt
    gui
    -L${ZLIB_HOME} -lz
    -lm -lpthread -ldl
    dbSta_lib
    grt_lib
    # utl
)