cmake_minimum_required(VERSION 2.9)
project(map_utils_py)

find_package(pybind11)
find_package(map_utils)
find_package(Boost)

set(CMAKE_CXX_STANDARD 17)

include_directories(
  ${geometry_utils_INCLUDE_DIR}
  ${map_utils_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

# pybind modules
pybind11_add_module(grid2d src/map_utils_py/grid2d.cpp)
target_link_libraries(grid2d PRIVATE ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(grid2d PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX})

pybind11_add_module(grid3d src/map_utils_py/grid3d.cpp)
target_link_libraries(grid3d PRIVATE ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(grid3d PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX})

install(TARGETS
  DESTINATION ${CMAKE_INSTALL_PREFIX})
