
ADD_LIBRARY(
    GLCD
    MCvMDistance.cpp
    MCvMDistanceAsym.cpp
    MCvMDistanceSym.cpp
    MCvMDistanceSymEven.cpp
    MCvMDistanceSymOdd.cpp
    MCvMDistanceOptimizer.cpp
    Misc.cpp
    ExpInt.cpp
    Computation.cpp
)

TARGET_LINK_LIBRARIES(
    GLCD
    Quadrature
    Optimization
)

INSTALL(TARGETS GLCD
        LIBRARY DESTINATION "lib/${TOOLBOX_FOLDER_NAME}")

INSTALL(DIRECTORY "."
        DESTINATION "include/${TOOLBOX_FOLDER_NAME}/GLCD"
        FILES_MATCHING PATTERN "*.h")

