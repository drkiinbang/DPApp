#include "BimInfo.h"
#include "Point.h"
#include "mbr.h"

#include <array>
#include <algorithm>
#include <iterator>

void BimInfo::addFace(Point normal, Point v1, Point v2, Point v3) {
  Face face(normal, v1, v2, v3);
  faces.push_back(face);
}

//MBR BimInfo::get_mbr(bool is_offset_applied, float* offset)
//{
//    //XYZ to{ std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
//    //XYZ from{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
//
//    XYZ to{ NUM_MIN, NUM_MIN, NUM_MIN };
//    XYZ from{ NUM_MAX, NUM_MAX, NUM_MAX };
//
//    for (std::size_t face_idx = 0; face_idx < faces.size(); face_idx++)
//    {
//        Face face = faces.at(face_idx);
//
//        for (std::size_t vertex_idx = 0; vertex_idx < 3; vertex_idx++)
//        {
//            Point point = face.getVertex(vertex_idx);
//            if (is_offset_applied)
//            {
//                point.setX(point.getX() - offset[0]);
//                point.setY(point.getY() - offset[1]);
//                point.setZ(point.getZ() - offset[2]);
//            }
//
//            if (from.x > point.getX()) from.x = point.getX();
//            if (to.x < point.getX()) to.x = point.getX();
//            if (from.y > point.getY()) from.y = point.getY();
//            if (to.y < point.getY()) to.y = point.getY();
//            if (from.z > point.getZ()) from.z = point.getZ();
//            if (to.z < point.getZ()) to.z = point.getZ();
//        }
//    }
//    MBR mbr(from, to);
//
//    return mbr;
//}

MBR BimInfo::get_mbr(const bool is_offset_applied, const float* offset)
{
  //XYZ to{ NUM_MIN, NUM_MIN, NUM_MIN };
  //XYZ from{ NUM_MAX, NUM_MAX, NUM_MAX };
  XYZ to{ std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
  XYZ from{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };

  // Iterate through all faces
  for (const auto& face : faces)
  {
    // Iterate through all vertices of the face
    for (std::size_t vertex_idx = 0; vertex_idx < 3; vertex_idx++)
    {
      Point point = face.getVertex(vertex_idx);

      // Apply offset if necessary
      if (is_offset_applied)
      {
        point.setX(point.getX() - offset[0]);
        point.setY(point.getY() - offset[1]);
        point.setZ(point.getZ() - offset[2]);
      }

      // Update MBR bounds
      from.x = std::min(from.x, point.getX());
      from.y = std::min(from.y, point.getY());
      from.z = std::min(from.z, point.getZ());

      to.x = std::max(to.x, point.getX());
      to.y = std::max(to.y, point.getY());
      to.z = std::max(to.z, point.getZ());
    }
  }

  MBR mbr(from, to);
  return mbr;
}

bool BimInfo::save(int id, std::string description, const bool is_offset_applied, const float* offset, std::ostream& os) {
    if (!os) {
        return false;
    }

    /// BUFFER_SIZE_RECORD_LEN
    /// ID_RECORD_LEN
    /// TRANSLATION_RECORD_LEN
    /// DESCRIPTION_RECORD_LEN
    /// faces.size() * VERTEX_POINT_RECORD_LEN * POINT_RECORD_LEN * COORD_LEN
    std::size_t buf_size = calculate_buffer_size();

    char* buffer = new char[buf_size];

    // 1. Buffer size
    reinterpret_cast<int*>(buffer)[0] = static_cast<int>(buf_size);
    // 2. Node ID
    reinterpret_cast<int*>(buffer)[1] = id;
    // 3. Translation
    std::size_t translate_pos = cacluate_translation_position();
    //for (int i = 0; i < 3; i++)
    //{
    //    memcpy(buffer + translate_pos + sizeof(float) * i, offset, sizeof(float) * 3);
    //}
    
    memcpy(buffer + translate_pos, offset, sizeof(float) * 3);
    
    // 4. Description
    memcpy(buffer + cacluate_description_position(), description.c_str(), DESCRIPTION_RECORD_LEN);
//#ifdef _WIN32
//    strcpy_s(buffer + cacluate_description_position(), DESCRIPTION_RECORD_LEN, info.c_str());    
//#else
//    strncpy(buffer + cacluate_description_position(), info.c_str(), DESCRIPTION_RECORD_LEN);
//#endif       

    // 5. Position Data
    std::size_t data_pos = calculate_data_position();

    for (std::size_t face_idx = 0; face_idx < faces.size(); face_idx++) 
    {
        Face face = faces.at(face_idx);
        std::size_t face_buffer_idx = data_pos + face_idx * VERTEX_POINT_RECORD_LEN * POINT_RECORD_LEN * COORD_LEN;

        for (std::size_t vertex_idx = 0; vertex_idx < 3; vertex_idx++) 
        {
            Point point = face.getVertex(vertex_idx);
            if (is_offset_applied)
            {
                point.setX(point.getX() - offset[0]);
                point.setY(point.getY() - offset[1]);
                point.setZ(point.getZ() - offset[2]);
            }
            memcpy(buffer + face_buffer_idx + vertex_idx * POINT_RECORD_LEN * COORD_LEN, point.getCoords(), POINT_RECORD_LEN * COORD_LEN);
        }
    }
    os.write(buffer, buf_size);

    delete[] buffer;

    return true;
}

bool BimInfo::saveTree(int id, std::string description, bool is_offset_applied, float* offset, std::ostream& os) {
    if (!os) {
        return false;
    }

    std::size_t buf_size = calculate_buffer_size();
    char* buffer = new char[buf_size];

    // 1. Buffer size
    reinterpret_cast<int*>(buffer)[0] = static_cast<int>(buf_size);
    // 2. Node ID
    reinterpret_cast<int*>(buffer)[1] = id;
    // 3. Translation
    std::size_t translate_pos = cacluate_translation_position();
    //for (int i = 0; i < 3; i++)
    //{
    //    memcpy(buffer + translate_pos + sizeof(float) * i, offset, sizeof(float) * 3);
    //}

    memcpy(buffer + translate_pos, offset, sizeof(float) * 3);

    // 4. Description
    memcpy(buffer + cacluate_description_position(), description.c_str(), DESCRIPTION_RECORD_LEN);
    //#ifdef _WIN32
    //    strcpy_s(buffer + cacluate_description_position(), DESCRIPTION_RECORD_LEN, info.c_str());    
    //#else
    //    strncpy(buffer + cacluate_description_position(), info.c_str(), DESCRIPTION_RECORD_LEN);
    //#endif       

        // 5. Position Data
    std::size_t data_pos = calculate_data_position();

    for (std::size_t face_idx = 0; face_idx < faces.size(); face_idx++)
    {
        Face face = faces.at(face_idx);
        std::size_t face_buffer_idx = data_pos + face_idx * VERTEX_POINT_RECORD_LEN * POINT_RECORD_LEN * COORD_LEN;

        for (std::size_t vertex_idx = 0; vertex_idx < 3; vertex_idx++)
        {
            Point point = face.getVertex(vertex_idx);
            if (is_offset_applied)
            {
                point.setX(point.getX() - offset[0]);
                point.setY(point.getY() - offset[1]);
                point.setZ(point.getZ() - offset[2]);
            }
            memcpy(buffer + face_buffer_idx + vertex_idx * POINT_RECORD_LEN * COORD_LEN, point.getCoords(), POINT_RECORD_LEN * COORD_LEN);
        }
    }
    os.write(buffer, buf_size);
    delete[] buffer;

    return true;
}

char* BimInfo::get_vertex_buffer(bool is_offset_applied, const float* offset, std::size_t& buffer_size)
{  
    std::size_t face_buffer_size = 3 * 3 * 4;
    buffer_size = faces.size() * 3 * 3 * 4;
    char* buffer = new char[buffer_size];
    for (std::size_t face_idx = 0; face_idx < faces.size(); face_idx++)
    {
        Face face = faces.at(face_idx);
        std::size_t face_buffer_idx = face_idx * VERTEX_POINT_RECORD_LEN * POINT_RECORD_LEN * COORD_LEN;
        for (std::size_t vertex_idx = 0; vertex_idx < 3; vertex_idx++)
        {
            Point point = face.getVertex(vertex_idx);
            if (is_offset_applied)
            {
                point.setX(point.getX() - offset[0]);
                point.setY(point.getY() - offset[1]);
                point.setZ(point.getZ() - offset[2]);
            }
            memcpy(buffer + face_buffer_idx + vertex_idx * POINT_RECORD_LEN * COORD_LEN, point.getCoords(), POINT_RECORD_LEN * COORD_LEN);
        }
    }
    return buffer;
}

std::size_t BimInfo::calculate_buffer_size() const
{
    std::size_t buffer_size = BUFFER_SIZE_RECORD_LEN + ID_RECORD_LEN + TRANSLATION_RECORD_LEN + DESCRIPTION_RECORD_LEN + faces.size() * VERTEX_POINT_RECORD_LEN * POINT_RECORD_LEN * COORD_LEN;
    return buffer_size;
}

std::size_t BimInfo::cacluate_translation_position() const 
{
    return BUFFER_SIZE_RECORD_LEN + ID_RECORD_LEN;
}

std::size_t BimInfo::cacluate_description_position() const
{
    return cacluate_translation_position() + TRANSLATION_RECORD_LEN;;
}

std::size_t BimInfo::calculate_data_position() const
{
    return cacluate_description_position() + DESCRIPTION_RECORD_LEN;
}