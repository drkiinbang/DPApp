#pragma once
#include <string>
#include <vector>
#include <iostream>

#include "face.h"
#include "mbr.h"

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