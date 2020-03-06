file(GLOB srcs "*.cpp" "*.hpp")

INCLUDE_DIRECTORIES(${JNI_INCLUDE_DIRS})

ADD_LIBRARY(dracoWrapper SHARED ${srcs})
target_link_libraries(dracoWrapper ${DART_LIBRARIES}
        DracoPnC
        ScorpioPnC
        worldNode
        myUtils
        dracoWrapper_swig)

add_executable(run_draco_wrapper ${srcs})
target_link_libraries(run_draco_wrapper ${DART_LIBRARIES}
        worldNode
        DracoPnC
        ScorpioPnC
        myUtils)