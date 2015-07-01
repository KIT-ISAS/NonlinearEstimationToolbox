
ADD_LIBRARY(
    Quadrature
    QAG.cpp
    Workspace.cpp
    GaussKronrod.cpp
)

INSTALL(TARGETS Quadrature
        LIBRARY DESTINATION "lib/${TOOLBOX_FOLDER_NAME}")

INSTALL(DIRECTORY "."
        DESTINATION "include/${TOOLBOX_FOLDER_NAME}/Quadrature"
        FILES_MATCHING PATTERN "*.h")

