# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/04-mmp_panda)
ADD_EXECUTABLE (controller04 controller.cpp)
ADD_EXECUTABLE (simviz04 simviz.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller04 ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz04 ${CS225A_COMMON_LIBRARIES})

# copy example interface to output
# FILE(COPY 04-mmp_panda.html DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY ../construction/mmp_panda.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY ../construction/world.urdf DESTINATION ${APP_RESOURCE_DIR})

# FILE(COPY ../model/YZ_steel DESTINATION ${APP_RESOURCE_DIR})

# copy in sai2-interfaces
FILE(COPY ${SAI2-INTERFACES_UI_DIR} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# copy launch script
FILE(COPY launch.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
