#pragma once

#ifdef BIMIMPORT_EXPORT
#define BIMIMPORT_API __declspec(dllexport)
#else
#define BIMIMPORT_API __declspec(dllimport)
#endif

#define API BIMIMPORT_API

#include <memory>
#include <string>
#include <vector>

#include "../include/bim/MeshChunk.h"

/// option 0: read all meshes
/// option 1: read mesheds in the nodes in a default scene
bool API loadGltf(const std::string& bim_folder, std::vector<mesh::MeshChunk>& bimData, const int option = 0);
bool API testGltfExtraId(const std::string& gltfPath);