file(GLOB srcs "*.cpp" "*.hpp")

add_executable(run_scorpio ${srcs})
target_link_libraries(run_scorpio ${DART_LIBRARIES}
                                  ScorpioPnC
                                  Scorpio2PnC
                                  DracoPnC
                                  myUtils)
