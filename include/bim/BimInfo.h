#pragma once
#include <string>
#include <vector>
#include <iostream>

#include "face.h"
#include "mbr.h"

/// 하나의 BIM 부재(요소)를 구성하는 face(삼각형) 목록과, 그것을 커스텀 바이너리
/// 포맷(.nodes2)으로 저장/조회하는 기능을 담당하는 클래스. 실제 구현(버퍼 레이아웃,
/// 오프셋 계산 등)은 BimInfo.cpp를 참고.
class BimInfo {
 public:
  BimInfo() = default;
  ~BimInfo() = default;

  void setInfo(std::string info) {
    this->info = info;
  }

  void addFace(Point normal, Point v1, Point v2, Point v3);

  auto &getFaces() {
    return faces;
  }

  std::size_t getFaceCount() {
    return faces.size();
  }

  MBR get_mbr(const bool is_offset_applied, const float* offset);

  bool save(int id, std::string description, const bool is_offset_applied, const float* offset, std::ostream& os);
  char* get_vertex_buffer(bool is_offset_applied, const float* offset, std::size_t& buffer_size);
  bool saveTree(int id, std::string description, bool is_offset_applied, float* offset, std::ostream& os);

 protected:
     std::size_t calculate_buffer_size() const;
     std::size_t cacluate_translation_position() const;
     std::size_t cacluate_description_position() const;
     std::size_t calculate_data_position() const;

private:
  std::string info;
  std::vector<Face> faces; 

  const int BUFFER_SIZE_RECORD_LEN = 4;
  const int ID_RECORD_LEN = 4;
  const int TRANSLATION_RECORD_LEN = 12;
  const int DESCRIPTION_RECORD_LEN = 200;
  const int VERTEX_POINT_RECORD_LEN = 3;
  const int POINT_RECORD_LEN = 3;
  const int COORD_LEN = 4;
};