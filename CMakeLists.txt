cmake_minimum_required(VERSION 3.15)
project(DPApp_II LANGUAGES CXX)

# This CMakeLists.txt is NOT maintained and does not build the current
# codebase. It predates a restructuring from src/master, src/slave,
# src/common into the current MasterApp/, SlaveApp/, src/NetworkManager.cpp
# layout, so its source globs (src/master/*.cpp, src/slave/*.cpp,
# src/common/*.cpp) matched no files at all.
#
# The only supported build is Visual Studio + DPApp.sln (Windows, x64).
# See doc/handover/03_빌드_가이드.md for the real build procedure.
#
# If a working CMake build is needed for this project, it has to be written
# from scratch against the current source layout (MasterApp/, SlaveApp/,
# BimImport/, LasImport/, src/, include/) and vendored dependencies
# (vendors/Eigen3, vendors/LAStools, vendors/gltf) -- that has not been done.
message(FATAL_ERROR
    "CMakeLists.txt is not maintained and will not produce a working build. "
    "Use Visual Studio + DPApp.sln instead (see doc/handover/03_빌드_가이드.md).")
