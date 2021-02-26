set(SIMULATION_LINK_LIBRARIES SPlisHSPlasH Utilities )

#eigen
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

############################################################
# NeighborhoodSearch
############################################################
include_directories(${NEIGHBORHOOD_SEARCH_INCLUDE_DIR})
set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_NeighborhoodSearch)
set(SIMULATION_LINK_LIBRARIES ${SIMULATION_LINK_LIBRARIES} ${NEIGHBORHOOD_SEARCH_LIBRARIES})

############################################################
# DiscreGrid
############################################################
include_directories(${CMAKE_BINARY_DIR}/extern/install/Discregrid/include)
set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_Discregrid)
set(SIMULATION_LINK_LIBRARIES ${SIMULATION_LINK_LIBRARIES}
	optimized Discregrid
	debug Discregrid_d)
link_directories(${CMAKE_BINARY_DIR}/extern/install/Discregrid/lib)

############################################################
# GenericParameters
############################################################
include_directories(${CMAKE_BINARY_DIR}/extern/install/GenericParameters/include)
set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_GenericParameters)

add_library(GazeboFluidSimulator SHARED
    ${PROJECT_SOURCE_DIR}/Simulator/GazeboWrapper/GazeboFluidSimulator.h
	${PROJECT_SOURCE_DIR}/Simulator/GazeboWrapper/GazeboFluidSimulator.cpp
	${PROJECT_SOURCE_DIR}/Simulator/GazeboWrapper/GazeboSceneLoader.cpp
	${PROJECT_SOURCE_DIR}/Simulator/GazeboWrapper/GazeboSceneLoader.h
	${PROJECT_SOURCE_DIR}/Simulator/GazeboWrapper/GazeboSimulatorBase.cpp
	${PROJECT_SOURCE_DIR}/Simulator/GazeboWrapper/GazeboSimulatorBase.h
)

if(DL_OUTPUT)
	add_definitions(-DDL_OUTPUT)
endif()

set_property(TARGET GazeboFluidSimulator PROPERTY POSITION_INDEPENDENT_CODE ON)

add_definitions(-DTW_NO_LIB_PRAGMA -DTW_STATIC)

set_target_properties(GazeboFluidSimulator PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(GazeboFluidSimulator PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(GazeboFluidSimulator PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
add_dependencies(GazeboFluidSimulator ${SIMULATION_DEPENDENCIES})

target_link_libraries(GazeboFluidSimulator ${SIMULATION_LINK_LIBRARIES} ${GAZEBO_LIBRARIES} ${IGNITION-MATH_LIBRARIES})

add_library(GazeboFluidVisualizer SHARED ${PROJECT_SOURCE_DIR}/Simulator/GUI/GazeboFluidVisualizer/GazeboFluidVisualizer.cpp)
target_link_libraries(GazeboFluidVisualizer gazebo_msgs ${OGRE_LIBRARIES} gazebo_gui)

set_target_properties(GazeboFluidSimulator PROPERTIES FOLDER "Simulator")

set(GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/lib/gazebo-${GAZEBO_MAJOR_VERSION}/plugins
)
install(DIRECTORY ${CMAKE_BINARY_DIR}/lib/ DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR} FILES_MATCHING PATTERN "libGazeboFluid*.so")
