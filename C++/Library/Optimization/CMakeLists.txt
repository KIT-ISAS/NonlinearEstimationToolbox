
ADD_LIBRARY(
    Optimization
    QuasiNewton.cpp
    BFGS.cpp
    LBFGS.cpp
    LineSearchNocedal.cpp
)

INSTALL(TARGETS Optimization
        LIBRARY DESTINATION "lib/${TOOLBOX_FOLDER_NAME}")

INSTALL(DIRECTORY "."
        DESTINATION "include/${TOOLBOX_FOLDER_NAME}/Optimization"
        FILES_MATCHING PATTERN "*.h")

