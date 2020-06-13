# This file is distributed under the MIT license.
# See the LICENSE file for details.

project(remote)

cmake_minimum_required(VERSION 2.8)

find_package(CUDA REQUIRED)

add_subdirectory(volkit)
include_directories(volkit/include/cpp)
include_directories(volkit/src/3rdparty/visionaray/include)
include_directories(volkit/src/3rdparty/visionaray/src)
include_directories(volkit/src/3rdparty/visionaray/src/3rdparty/CmdLine/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/volkit/src/3rdparty/visionaray/config)

cuda_add_executable(client client.cpp sphere_renderer.cu)
target_link_libraries(client volkit visionaray visionaray_common)

cuda_add_executable(server server.cpp volume_renderer.cu)
target_link_libraries(server volkit visionaray visionaray_common)
