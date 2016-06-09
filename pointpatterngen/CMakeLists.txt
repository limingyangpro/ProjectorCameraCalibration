# directories

add_executable(pointpatterngen pointpatterngen.cpp)

# linking other libraries
target_link_libraries(
    pointpatterngen
    ${OpenCV_LIBS}
)

add_executable(chessboardgenerator chessboardgenerator.cpp)

# linking other libraries
target_link_libraries(
    chessboardgenerator
    ${OpenCV_LIBS}
)

add_executable(acirclegridgenerator acirclegridgenerator.cpp)

# linking other libraries
target_link_libraries(
    acirclegridgenerator
    ${OpenCV_LIBS}
)
