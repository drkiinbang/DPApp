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

/// bim_folder 안의 모든 GLTF/GLB 파일을 읽어 chunkbim::MeshChunk 목록(bimData)으로 반환한다.
/// [참고] 과거에는 "option 0: 모든 메시 읽기 / option 1: 기본 씬 노드의 메시만 읽기"처럼
/// 옵션으로 두 가지 읽기 방식을 선택했던 것으로 보이나, 현재 시그니처에는 그런 option
/// 매개변수가 없다 — BimImport.cpp의 실제 구현(loadGltf)은 항상 collect_gltf_files()로
/// 폴더 내 모든 GLTF/GLB 파일을 모아 전부 읽어들인다.
bool API loadGltf(const std::string& bim_folder, std::vector<chunkbim::MeshChunk>& bimData);
bool API testGltfExtraId(const std::string& gltfPath);