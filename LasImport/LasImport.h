#pragma once

#ifdef LASIMPORT_EXPORT
#define LASIMPORT_API __declspec(dllexport)
#else
#define LASIMPORT_API __declspec(dllimport)
#endif

#define API LASIMPORT_API

#include <string>

#include "../include/bim/MeshChunk.h"

/// LAS 포인트클라우드 파일을 읽어 chunkpc::PointCloudChunk 목록으로 분할 적재하는
/// 별도의 DLL 모듈. DLL 경계를 넘나드는 API이므로 매개변수는 기본 타입/표준 컨테이너로만
/// 구성되어 있다 (LASIMPORT_EXPORT가 정의된 쪽에서 빌드하면 __declspec(dllexport),
/// 그 외(이 DLL을 사용하는 쪽)에서는 __declspec(dllimport)로 선언된다).
/// max_points_per_chunk 단위로 나눠 담는 이유는 한 청크가 네트워크 전송 단위(TaskManager의
/// PointCloudChunk)와 맞물리기 때문이다.
bool API loadLasFile(const std::string& file_path, std::vector<chunkpc::PointCloudChunk>& chunks, const uint32_t max_points_per_chunk);