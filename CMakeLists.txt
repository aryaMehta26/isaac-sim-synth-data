cmake_minimum_required(VERSION 3.14)
project(IsaacSimSynthData LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find USD/Omniverse Packages
# In a real Isaac Sim C++ env, we would find_package(PXR REQUIRED) etc.
# For this "standalone" representation, we will assume paths or stub them.
# include_directories(${PXR_INCLUDE_DIRS})
# link_directories(${PXR_LIBRARY_DIRS})

include_directories(include)

add_executable(isaac_synth
    src/main.cpp
    src/SceneBuilder.cpp
    src/SensorManager.cpp
    src/Randomizer.cpp
    src/DataExporter.cpp
)

# target_link_libraries(isaac_synth PRIVATE ${PXR_LIBRARIES})
