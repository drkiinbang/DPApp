# DPApp_II Makefile
#
# This Makefile is NOT maintained and does not build the current codebase.
# It predates a restructuring from src/master, src/slave, src/common into the
# current MasterApp/, SlaveApp/, src/NetworkManager.cpp layout, so its source
# globs (src/master/*.cpp, src/slave/*.cpp, src/common/*.cpp) match no files
# at all today. `make all` used to silently produce empty/broken binaries.
#
# The only supported build is Visual Studio + DPApp.sln (Windows, x64).
# See doc/handover/03_빌드_가이드.md for the real build procedure.
#
# If a working make/CMake-based build is needed for this project, it has to be
# written from scratch against the current source layout (MasterApp/,
# SlaveApp/, BimImport/, LasImport/, src/, include/) and vendored dependencies
# (vendors/Eigen3, vendors/LAStools, vendors/gltf) -- that has not been done.

.PHONY: all master slave test clean rebuild install help

all master slave test clean rebuild install:
	@echo "This Makefile is not maintained and will not produce a working build."
	@echo "Use Visual Studio + DPApp.sln instead (see doc/handover/03_빌드_가이드.md)."
	@exit 1

help:
	@echo "This Makefile is not maintained. See the header comment in this file."
