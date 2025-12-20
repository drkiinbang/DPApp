// inet_ntoa 경고 해결
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include "../include/NetworkManager.h"
#include "../include/Logger.h"
#include "../include/BinaryReadWriter.hpp"

#include <iostream>
#include <sstream>
#include <random>
#include <chrono>
#include <cstring>

namespace DPApp {

    const int heartbeat_interval = 10;

    //========================================
    // NetworkMessage Implementation
    //========================================
    NetworkMessage::NetworkMessage(MessageType type, std::vector<uint8_t> payload)
        : data(std::move(payload)) { /// [Fix] Move payload into member variable
        header.type = type;
        header.data_length = static_cast<uint32_t>(data.size()); // use member .size()
        calculateChecksum();
    }

    std::vector<uint8_t> NetworkMessage::serialize() const {
        std::vector<uint8_t> buffer(sizeof(MessageHeader) + data.size());
        std::memcpy(buffer.data(), &header, sizeof(MessageHeader));
        if (!data.empty()) {
            std::memcpy(buffer.data() + sizeof(MessageHeader), data.data(), data.size());
        }
        return buffer;
    }

    NetworkMessage NetworkMessage::deserialize(const std::vector<uint8_t>& header_buffer, std::vector<uint8_t> data_buffer) {
        if (header_buffer.size() < sizeof(MessageHeader)) {
            throw std::runtime_error("Buffer too small for message header");
        }

        NetworkMessage message;
        std::memcpy(&message.header, header_buffer.data(), sizeof(MessageHeader));

        if (message.header.magic != MessageHeader::MAGIC_NUMBER) {
            throw std::runtime_error("Invalid magic number");
        }

        message.data = std::move(data_buffer);

        if (message.header.data_length != message.data.size()) {
            throw std::runtime_error("Data length mismatch in header vs actual data");
        }

        if (!message.verifyChecksum()) {
            throw std::runtime_error("Checksum verification failed");
        }

        return message;
    }

    void NetworkMessage::calculateChecksum() {
        header.checksum = NetworkUtils::calculateCRC32(data);
    }

    bool NetworkMessage::verifyChecksum() const {
        return header.checksum == NetworkUtils::calculateCRC32(data);
    }


    //========================================
    // NetworkUtils Implementation
    //========================================
    namespace NetworkUtils {

        bool recvAll(int socket_fd, std::vector<uint8_t>& buffer, size_t bytes_to_receive) {
            if (bytes_to_receive == 0) {
                buffer.clear();
                return true;
            }
            buffer.resize(bytes_to_receive);
            size_t total_bytes_received = 0;

            while (total_bytes_received < bytes_to_receive) {
                /// Fix char* casting and clarify remaining byte calculation
                int bytes_received = recv(socket_fd,
                    reinterpret_cast<char*>(buffer.data()) + total_bytes_received,
                    static_cast<int>(bytes_to_receive - total_bytes_received), 0);

                if (bytes_received == 0) {
                    ILOG << "Disconnected: bytes_received == 0";
                    return false;
                }
                else if (bytes_received < 0) {
#ifdef _WIN32
                    int error = WSAGetLastError();
                    if (error == WSAEWOULDBLOCK || error == WSAEINTR) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
#else
                    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
#endif
                    /// Provide more detailed logging
                    ILOG << "Disconnected: error code " << error;
                    return false;
                }
                total_bytes_received += bytes_received;
            }
            return true;
        }

        bool sendAll(int socket_fd, const std::vector<uint8_t>& buffer) {
            if (buffer.empty()) return true;
            size_t total_bytes_sent = 0;
            while (total_bytes_sent < buffer.size()) {
                int bytes_sent = send(socket_fd, reinterpret_cast<const char*>(buffer.data()) + total_bytes_sent, static_cast<int>(buffer.size() - total_bytes_sent), 0);
                if (bytes_sent < 0) return false;
                total_bytes_sent += bytes_sent;
            }
            return true;
        }

        std::string generateSlaveId() {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            static std::uniform_int_distribution<> dis(0, 15);
            std::stringstream ss;
            ss << "slave_";
            for (int i = 0; i < 8; ++i) ss << std::hex << dis(gen);
            return ss.str();
        }

        uint32_t calculateCRC32(const std::vector<uint8_t>& data) {
            uint32_t crc = 0xFFFFFFFF;
            for (uint8_t byte : data) {
                crc ^= byte;
                for (int i = 0; i < 8; ++i) {
                    crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320) : (crc >> 1);
                }
            }
            return ~crc;
        }

        /// ================================================================
        /// Task Serialization
        /// ================================================================
        std::vector<uint8_t> serializeTask(const ProcessingTask& task) {
            std::vector<uint8_t> data;
            BinaryWriter writer(data);

            /// 1. Task ID and Chunk ID
            writer.write(task.task_id);
            writer.write(task.chunk_id);

            /// 2. Task Type
            writer.write(static_cast<uint8_t>(task.task_type));

            /// 3. Parameters (Batch Write)
            uint64_t param_len = static_cast<uint64_t>(task.parameters.size());
            writer.write(param_len);

            if (param_len > 0) {
                /// Write the entire parameter buffer at once
                writer.writeBytes(task.parameters.data(), static_cast<size_t>(param_len));
            }

            return data;
        }

        /// ================================================================
        /// Task Deserialization
        /// ================================================================
        ProcessingTask deserializeTask(const std::vector<uint8_t>& data) {
            BinaryReader reader(data);
            ProcessingTask task;

            /// 1. Task ID and Chunk ID
            task.task_id = reader.read<uint32_t>();
            task.chunk_id = reader.read<uint32_t>();

            /// 2. Task Type
            task.task_type = static_cast<TaskType>(reader.read<uint8_t>());

            /// 3. Parameters (Batch Read)
            uint64_t param_len = reader.read<uint64_t>();
            if (param_len > 0) {
                /// Get a direct pointer to the data buffer (Zero-copy or single memcpy)
                const void* ptr = reader.readBytesRef(static_cast<size_t>(param_len));
                const uint8_t* byte_ptr = static_cast<const uint8_t*>(ptr);
                task.parameters.assign(byte_ptr, byte_ptr + param_len);
            }

            return task;
        }

        /// ================================================================
        /// Result Serialization
        /// ================================================================
        std::vector<uint8_t> serializeResult(const ProcessingResult& result) {
            std::vector<uint8_t> data;
            BinaryWriter writer(data);

            /// 1. Headers
            writer.write(result.task_id);
            writer.write(result.chunk_id);
            writer.write(static_cast<uint8_t>(result.success ? 1 : 0));

            uint64_t error_msg_len = static_cast<uint64_t>(result.error_message.length());
            uint64_t points_len = static_cast<uint64_t>(result.processed_points.size());
            uint64_t result_data_len = static_cast<uint64_t>(result.result_data.size());

            writer.write(error_msg_len);
            writer.write(points_len);
            writer.write(result_data_len);

            /// 2. Error Message
            if (error_msg_len > 0) {
                writer.writeBytes(result.error_message.data(), static_cast<size_t>(error_msg_len));
            }

            /// 3. Processed Points (Optimized Batch Write)
            if (points_len > 0) {
                /// Since Point3D might not be a POD type (contains vector), we flatten it first.
                /// Prepare a contiguous float buffer [x, y, z, x, y, z, ...]
                std::vector<float> raw_floats;
                raw_floats.reserve(points_len * 3);
                for (const auto& point : result.processed_points) {
                    raw_floats.push_back(point.x());
                    raw_floats.push_back(point.y());
                    raw_floats.push_back(point.z());
                }
                /// Write all floats in one go (fastest on Intel/AMD)
                writer.writeBytes(raw_floats.data(), raw_floats.size() * sizeof(float));
            }

            /// 4. Result Data
            if (result_data_len > 0) {
                writer.writeBytes(result.result_data.data(), static_cast<size_t>(result_data_len));
            }

            return data;
        }

        /// ================================================================
        /// Result Deserialization
        /// ================================================================
        ProcessingResult deserializeResult(const std::vector<uint8_t>& data) {
            BinaryReader reader(data);
            ProcessingResult result;

            /// 1. Headers
            result.task_id = reader.read<uint32_t>();
            result.chunk_id = reader.read<uint32_t>();
            result.success = (reader.read<uint8_t>() != 0);

            uint64_t error_msg_len = reader.read<uint64_t>();
            uint64_t points_len = reader.read<uint64_t>();
            uint64_t result_data_len = reader.read<uint64_t>();

            /// 2. Error Message
            if (error_msg_len > 0) {
                const void* ptr = reader.readBytesRef(static_cast<size_t>(error_msg_len));
                result.error_message.assign(static_cast<const char*>(ptr), static_cast<size_t>(error_msg_len));
            }

            /// 3. Processed Points (Optimized Batch Read)
            if (points_len > 0) {
                result.processed_points.resize(static_cast<size_t>(points_len));

                /// Calculate total size for all points (3 floats per point)
                size_t total_floats = static_cast<size_t>(points_len) * 3;
                size_t total_bytes = total_floats * sizeof(float);

                /// Get direct pointer to raw memory (No per-point function call overhead)
                /// Since we are on Intel/AMD (Little Endian), we can cast bytes directly to float*
                const uint8_t* raw_bytes = static_cast<const uint8_t*>(reader.readBytesRef(total_bytes));

                for (size_t i = 0; i < static_cast<size_t>(points_len); ++i) {
                    float x, y, z;
                    std::memcpy(&x, raw_bytes + (i * 3 + 0) * sizeof(float), sizeof(float));
                    std::memcpy(&y, raw_bytes + (i * 3 + 1) * sizeof(float), sizeof(float));
                    std::memcpy(&z, raw_bytes + (i * 3 + 2) * sizeof(float), sizeof(float));
                    result.processed_points[i] = Point3D(x, y, z);
                }
            }

            /// 4. Result Data
            if (result_data_len > 0) {
                const void* ptr = reader.readBytesRef(static_cast<size_t>(result_data_len));
                const uint8_t* byte_ptr = static_cast<const uint8_t*>(ptr);
                result.result_data.assign(byte_ptr, byte_ptr + result_data_len);
            }

            return result;
        }

        /// ================================================================
        /// Chunk Serialization
        /// ================================================================
        std::vector<uint8_t> serializeChunk(const PointCloudChunk& chunk) {
            std::vector<uint8_t> data;
            BinaryWriter writer(data);

            /// 1. Chunk Metadata
            writer.write(chunk.chunk_id);
            uint64_t points_count = static_cast<uint64_t>(chunk.points.size());
            writer.write(points_count);

            writer.write(chunk.min_x); writer.write(chunk.min_y); writer.write(chunk.min_z);
            writer.write(chunk.max_x); writer.write(chunk.max_y); writer.write(chunk.max_z);

            uint64_t filename_len = static_cast<uint64_t>(chunk.header.filename.length());
            writer.write(filename_len);
            if (filename_len > 0) {
                writer.writeBytes(chunk.header.filename.data(), static_cast<size_t>(filename_len));
            }

            /// 2. Points Data (Optimized Batch Write)
            if (points_count > 0) {
                /// Flatten point data into a continuous float buffer
                std::vector<float> raw_floats;
                raw_floats.reserve(points_count * 3);
                for (const auto& point : chunk.points) {
                    raw_floats.push_back(point.x());
                    raw_floats.push_back(point.y());
                    raw_floats.push_back(point.z());
                }
                /// Bulk write to the buffer
                writer.writeBytes(raw_floats.data(), raw_floats.size() * sizeof(float));
            }

            return data;
        }

        /// ================================================================
        /// Chunk Deserialization
        /// ================================================================
        PointCloudChunk deserializeChunk(const std::vector<uint8_t>& data) {
            BinaryReader reader(data);
            PointCloudChunk chunk;

            /// 1. Chunk Metadata
            chunk.chunk_id = reader.read<uint32_t>();
            uint64_t points_count = reader.read<uint64_t>();

            chunk.min_x = reader.read<float>(); chunk.min_y = reader.read<float>(); chunk.min_z = reader.read<float>();
            chunk.max_x = reader.read<float>(); chunk.max_y = reader.read<float>(); chunk.max_z = reader.read<float>();

            uint64_t filename_len = reader.read<uint64_t>();
            if (filename_len > 0) {
                const void* ptr = reader.readBytesRef(static_cast<size_t>(filename_len));
                chunk.header.filename.assign(static_cast<const char*>(ptr), static_cast<size_t>(filename_len));
            }

            /// 2. Points Data (Optimized Batch Read)
            if (points_count > 0) {
                chunk.points.resize(static_cast<size_t>(points_count));

                size_t total_floats = static_cast<size_t>(points_count) * 3;
                size_t total_bytes = total_floats * sizeof(float);

                /// Get pointer to raw bytes and cast to float*
                /// This assumes Little Endian architecture (Intel/AMD standard)
                const uint8_t* raw_bytes = static_cast<const uint8_t*>(reader.readBytesRef(total_bytes));

                for (size_t i = 0; i < static_cast<size_t>(points_count); ++i) {
                    float x, y, z;
                    std::memcpy(&x, raw_bytes + (i * 3 + 0) * sizeof(float), sizeof(float));
                    std::memcpy(&y, raw_bytes + (i * 3 + 1) * sizeof(float), sizeof(float));
                    std::memcpy(&z, raw_bytes + (i * 3 + 2) * sizeof(float), sizeof(float));
                    chunk.points[i] = Point3D(x, y, z);
                }
            }

            return chunk;
        }

        /// ================================================================
        /// MeshChunk Serialization (Optimized Batch Write)
        /// ================================================================
        std::vector<uint8_t> serializeMeshChunk(const chunkbim::MeshChunk& meshChunk) {
            std::vector<uint8_t> data;
            BinaryWriter writer(data);

            /// 1. ID and Name
            writer.write(meshChunk.id);
            writer.writeString(meshChunk.name);

            /// 2. Vertices (Batch Write)
            uint32_t vertex_count = static_cast<uint32_t>(meshChunk.vertices.size());
            writer.write(vertex_count);

            if (vertex_count > 0) {
                /// Flatten XYZPoint (non-POD due to internal vector) to raw floats
                std::vector<float> raw_vertices;
                raw_vertices.reserve(vertex_count * 3);
                for (const auto& vtx : meshChunk.vertices) {
                    raw_vertices.push_back(vtx.x());
                    raw_vertices.push_back(vtx.y());
                    raw_vertices.push_back(vtx.z());
                }
                writer.writeBytes(raw_vertices.data(), raw_vertices.size() * sizeof(float));
            }

            /// 3. Faces (Batch Write)
            uint32_t face_count = static_cast<uint32_t>(meshChunk.faces.size());
            writer.write(face_count);

            if (face_count > 0) {
                /// Separate normals and indices for efficient batch writing
                std::vector<float> raw_normals;
                std::vector<unsigned int> raw_indices;

                raw_normals.reserve(face_count * 3);
                raw_indices.reserve(face_count * 3);

                for (const auto& face : meshChunk.faces) {
                    /// Normals
                    raw_normals.push_back(face.normal.x());
                    raw_normals.push_back(face.normal.y());
                    raw_normals.push_back(face.normal.z());

                    /// Indices
                    raw_indices.push_back(face.idxVtx[0]);
                    raw_indices.push_back(face.idxVtx[1]);
                    raw_indices.push_back(face.idxVtx[2]);
                }

                writer.writeBytes(raw_normals.data(), raw_normals.size() * sizeof(float));
                writer.writeBytes(raw_indices.data(), raw_indices.size() * sizeof(unsigned int));
            }

            /// 4. Bounding box
            writer.write(meshChunk.min_x); writer.write(meshChunk.min_y); writer.write(meshChunk.min_z);
            writer.write(meshChunk.max_x); writer.write(meshChunk.max_y); writer.write(meshChunk.max_z);

            return data;
        }

        /// ================================================================
        /// MeshChunk Deserialization (Optimized Batch Read)
        /// ================================================================
        chunkbim::MeshChunk deserializeMeshChunk(const std::vector<uint8_t>& data) {
            BinaryReader reader(data);
            chunkbim::MeshChunk meshChunk;

            /// 1. ID and Name
            meshChunk.id = reader.read<int>();
            meshChunk.name = reader.readString();

            /// 2. Vertices (Batch Read)
            uint32_t vertex_count = reader.read<uint32_t>();

            if (vertex_count > 0) {
                meshChunk.vertices.resize(vertex_count);
                size_t total_bytes = static_cast<size_t>(vertex_count) * 3 * sizeof(float);

                /// Direct memory access
                const uint8_t* raw_bytes = static_cast<const uint8_t*>(reader.readBytesRef(total_bytes));

                for (uint32_t i = 0; i < vertex_count; ++i) {
                    float x, y, z;
                    std::memcpy(&x, raw_bytes + (i * 3 + 0) * sizeof(float), sizeof(float));
                    std::memcpy(&y, raw_bytes + (i * 3 + 1) * sizeof(float), sizeof(float));
                    std::memcpy(&z, raw_bytes + (i * 3 + 2) * sizeof(float), sizeof(float));
                    meshChunk.vertices[i] = pctree::XYZPoint(x, y, z);
                }
            }

            /// 3. Faces (Batch Read)
            uint32_t face_count = reader.read<uint32_t>();

            if (face_count > 0) {
                meshChunk.faces.resize(face_count);

                /// Read Normals (Batch)
                size_t normals_bytes = static_cast<size_t>(face_count) * 3 * sizeof(float);
                const uint8_t* raw_normal_bytes = static_cast<const uint8_t*>(reader.readBytesRef(normals_bytes));

                /// Read Indices (Batch)
                size_t indices_bytes = static_cast<size_t>(face_count) * 3 * sizeof(unsigned int);
                const uint8_t* raw_index_bytes = static_cast<const uint8_t*>(reader.readBytesRef(indices_bytes));

                for (uint32_t i = 0; i < face_count; ++i) {
                    float nx, ny, nz;
                    std::memcpy(&nx, raw_normal_bytes + (i * 3 + 0) * sizeof(float), sizeof(float));
                    std::memcpy(&ny, raw_normal_bytes + (i * 3 + 1) * sizeof(float), sizeof(float));
                    std::memcpy(&nz, raw_normal_bytes + (i * 3 + 2) * sizeof(float), sizeof(float));
                    meshChunk.faces[i].normal = pctree::XYZPoint(nx, ny, nz);

                    unsigned int idx0, idx1, idx2;
                    std::memcpy(&idx0, raw_index_bytes + (i * 3 + 0) * sizeof(unsigned int), sizeof(unsigned int));
                    std::memcpy(&idx1, raw_index_bytes + (i * 3 + 1) * sizeof(unsigned int), sizeof(unsigned int));
                    std::memcpy(&idx2, raw_index_bytes + (i * 3 + 2) * sizeof(unsigned int), sizeof(unsigned int));
                    meshChunk.faces[i].idxVtx[0] = idx0;
                    meshChunk.faces[i].idxVtx[1] = idx1;
                    meshChunk.faces[i].idxVtx[2] = idx2;
                }
            }

            /// 4. Bounding box
            meshChunk.min_x = reader.read<float>(); meshChunk.min_y = reader.read<float>(); meshChunk.min_z = reader.read<float>();
            meshChunk.max_x = reader.read<float>(); meshChunk.max_y = reader.read<float>(); meshChunk.max_z = reader.read<float>();

            return meshChunk;
        }

        /// ================================================================
        /// BimPcChunk Serialization (Reusing optimized logic)
        /// ================================================================
        std::vector<uint8_t> serializeBimPcChunk(const BimPcChunk& chunk) {
            std::vector<uint8_t> data;
            BinaryWriter writer(data);

            /// 1. Meta data
            writer.write(chunk.chunk_id);

            /// 2. Point Cloud (Optimized Batch Write)
            uint32_t point_count = static_cast<uint32_t>(chunk.points.size());
            writer.write(point_count);

            if (point_count > 0) {
                std::vector<float> raw_floats;
                raw_floats.reserve(point_count * 3);
                for (const auto& point : chunk.points) {
                    raw_floats.push_back(point.x());
                    raw_floats.push_back(point.y());
                    raw_floats.push_back(point.z());
                }
                writer.writeBytes(raw_floats.data(), raw_floats.size() * sizeof(float));
            }

            /// 3. Bounding Box
            writer.write(chunk.min_x); writer.write(chunk.min_y); writer.write(chunk.min_z);
            writer.write(chunk.max_x); writer.write(chunk.max_y); writer.write(chunk.max_z);

            /// 4. MeshChunk (Nested serialization)
            std::vector<uint8_t> bim_data = serializeMeshChunk(chunk.bim);
            uint32_t bim_data_size = static_cast<uint32_t>(bim_data.size());

            writer.write(bim_data_size);
            writer.writeBytes(bim_data.data(), bim_data_size);

            return data;
        }

        /// ================================================================
        /// BimPcChunk Deserialization (Reusing optimized logic)
        /// ================================================================
        BimPcChunk deserializeBimPcChunk(const std::vector<uint8_t>& data) {
            BinaryReader reader(data);
            BimPcChunk chunk;

            /// 1. Meta data
            chunk.chunk_id = reader.read<uint32_t>();

            /// 2. Point Cloud (Optimized Batch Read)
            uint32_t point_count = reader.read<uint32_t>();

            if (point_count > 0) {
                chunk.points.resize(point_count);
                size_t total_bytes = static_cast<size_t>(point_count) * 3 * sizeof(float);

                /// Batch read floats directly from buffer
                const float* raw_floats = static_cast<const float*>(reader.readBytesRef(total_bytes));

                for (uint32_t i = 0; i < point_count; ++i) {
                    chunk.points[i] = Point3D(
                        raw_floats[i * 3 + 0],
                        raw_floats[i * 3 + 1],
                        raw_floats[i * 3 + 2]
                    );
                }
            }

            /// 3. Bounding Box
            chunk.min_x = reader.read<float>(); chunk.min_y = reader.read<float>(); chunk.min_z = reader.read<float>();
            chunk.max_x = reader.read<float>(); chunk.max_y = reader.read<float>(); chunk.max_z = reader.read<float>();

            /// 4. MeshChunk (Nested deserialization)
            uint32_t bim_data_size = reader.read<uint32_t>();
            const void* bim_ptr = reader.readBytesRef(bim_data_size);
            const uint8_t* byte_ptr = static_cast<const uint8_t*>(bim_ptr);
            std::vector<uint8_t> bim_buffer(byte_ptr, byte_ptr + bim_data_size);
            chunk.bim = deserializeMeshChunk(bim_buffer);

            return chunk;
        }

        /// ================================================================
        /// BimPcResult Serialization (Optimized Batch Write)
        /// ================================================================
        std::vector<uint8_t> serializeBimPcResult(const BimPcResult& result) {
            std::vector<uint8_t> data;
            BinaryWriter writer(data);

            /// 1. Task info and Success flag
            writer.write(result.task_id);
            writer.write(result.chunk_id);
            writer.write(static_cast<uint8_t>(result.success ? 1 : 0));

            /// 2. Error Message
            writer.writeString(result.error_message);

            /// 3. Statistics
            writer.write(result.total_points_processed);
            writer.write(result.total_faces_processed);
            writer.write(result.processing_time_ms);

            /// 4. Distance results
            writer.write(result.min_distance);
            writer.write(result.max_distance);
            writer.write(result.avg_distance);
            writer.write(result.std_deviation);

            /// 5. Point Distances (Batch Write - vector<double> is continuous)
            uint32_t dist_count = static_cast<uint32_t>(result.point_distances.size());
            writer.write(dist_count);
            if (dist_count > 0) {
                writer.writeBytes(result.point_distances.data(), dist_count * sizeof(double));
            }

            /// 6. Nearest Face IDs (Batch Write - vector<int32_t> is continuous)
            uint32_t face_id_count = static_cast<uint32_t>(result.nearest_face_ids.size());
            writer.write(face_id_count);
            if (face_id_count > 0) {
                writer.writeBytes(result.nearest_face_ids.data(), face_id_count * sizeof(int32_t));
            }

            /// 7. Classification
            writer.write(result.points_within_threshold);
            writer.write(result.points_outside_threshold);

            /// 8. Extra Data
            uint32_t extra_len = static_cast<uint32_t>(result.extra_data.size());
            writer.write(extra_len);
            if (extra_len > 0) {
                writer.writeBytes(result.extra_data.data(), extra_len);
            }

            return data;
        }

        /// ================================================================
        /// BimPcResult Deserialization (Optimized Batch Read)
        /// ================================================================
        BimPcResult deserializeBimPcResult(const std::vector<uint8_t>& data) {
            BinaryReader reader(data);
            BimPcResult result;

            /// 1. Task info and Success flag
            result.task_id = reader.read<uint32_t>();
            result.chunk_id = reader.read<uint32_t>();
            result.success = (reader.read<uint8_t>() != 0);

            /// 2. Error Message
            result.error_message = reader.readString();

            /// 3. Statistics
            result.total_points_processed = reader.read<uint32_t>();
            result.total_faces_processed = reader.read<uint32_t>();
            result.processing_time_ms = reader.read<double>();

            /// 4. Distance results
            result.min_distance = reader.read<double>();
            result.max_distance = reader.read<double>();
            result.avg_distance = reader.read<double>();
            result.std_deviation = reader.read<double>();

            /// 5. Point Distances (Batch Read)
            uint32_t dist_count = reader.read<uint32_t>();
            if (dist_count > 0) {
                size_t bytes = dist_count * sizeof(double);
                const void* ptr = reader.readBytesRef(bytes);

                result.point_distances.resize(dist_count);
                /// Simple memcpy is safe here because double is a POD type
                std::memcpy(result.point_distances.data(), ptr, bytes);
            }

            /// 6. Nearest Face IDs (Batch Read)
            uint32_t face_id_count = reader.read<uint32_t>();
            if (face_id_count > 0) {
                size_t bytes = face_id_count * sizeof(int32_t);
                const void* ptr = reader.readBytesRef(bytes);

                result.nearest_face_ids.resize(face_id_count);
                std::memcpy(result.nearest_face_ids.data(), ptr, bytes);
            }

            /// 7. Classification
            result.points_within_threshold = reader.read<uint32_t>();
            result.points_outside_threshold = reader.read<uint32_t>();

            /// 8. Extra Data
            uint32_t extra_len = reader.read<uint32_t>();
            if (extra_len > 0) {
                const void* ptr = reader.readBytesRef(extra_len);
                const uint8_t* byte_ptr = static_cast<const uint8_t*>(ptr);
                result.extra_data.assign(byte_ptr, byte_ptr + extra_len);
            }

            return result;
        }


    } // namespace NetworkUtils

    //========================================
    // NetworkServer Implementation
    //========================================
    NetworkServer::NetworkServer() : server_socket_(-1), port_(0), running_(false), shutdown_requested_(false) {
        cfg_ = DPApp::RuntimeConfig::loadFromEnv();
        worker_pool_ = std::make_unique<DPApp::ThreadPool>(static_cast<size_t>(cfg_.server_io_threads));
#ifdef _WIN32
        WSADATA wsaData; WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    }

    NetworkServer::~NetworkServer() {
        stop();
#ifdef _WIN32
        WSACleanup();
#endif
    }

    bool NetworkServer::start(uint16_t port) {
        if (running_) return false;
        port_ = port;
        if (!initializeSocket()) return false;
        running_ = true;
        shutdown_requested_ = false;
        server_thread_ = std::thread(&NetworkServer::serverThread, this);
        heartbeat_thread_ = std::thread(&NetworkServer::heartbeatThread, this);
        ILOG << "Server started on port " << port;
        return true;
    }

    void NetworkServer::stop() {
        if (!running_) return;
        shutdown_requested_ = true;
        running_ = false;
        cleanupSocket();
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (auto& [id, client] : clients_) {
                client->is_active = false;
#ifdef _WIN32
                shutdown(client->socket_fd, SD_BOTH);
                closesocket(client->socket_fd);
#else
                shutdown(client->socket_fd, SHUT_RDWR);
                close(client->socket_fd);
#endif
            }
            clients_.clear();
        }
        if (server_thread_.joinable()) server_thread_.join();
        if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
        ILOG << "Server stopped";
    }

    bool NetworkServer::initializeSocket() {
        server_socket_ = static_cast<int>(socket(AF_INET, SOCK_STREAM, 0));
        if (server_socket_ < 0) { ELOG << "Failed to create socket"; return false; }
        int opt = 1;
        setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port_);
        if (bind(server_socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) { ELOG << "Failed to bind socket"; cleanupSocket(); return false; }
        if (listen(server_socket_, cfg_.server_backlog) < 0) { ELOG << "Failed to listen"; cleanupSocket(); return false; }
        return true;
    }

    void NetworkServer::cleanupSocket() {
        if (server_socket_ >= 0) {
#ifdef _WIN32
            closesocket(server_socket_);
#else
            close(server_socket_);
#endif
            server_socket_ = -1;
        }
    }

    void NetworkServer::serverThread() {
        while (running_) {
            sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int client_socket = static_cast<int>(accept(server_socket_, reinterpret_cast<sockaddr*>(&client_addr), &client_len));
            if (client_socket < 0) {
                if (running_) ELOG << "Failed to accept client connection";
                continue;
            }
            auto client = std::make_shared<ClientConnection>(client_socket, sockaddrToString(client_addr), ntohs(client_addr.sin_port));
            client->slave_id = "pending_" + NetworkUtils::generateSlaveId();
            worker_pool_->enqueue([this, client]() { this->clientHandlerThread(client); });
        }
    }

    void NetworkServer::clientHandlerThread(std::shared_ptr<ClientConnection> client) {
        std::string current_client_id = client->slave_id;
        ILOG << "Client handler started [ID: " << current_client_id << ", Address: " << client->address << ":" << client->port << "]";
        while (client->is_active && running_) {
            try {
                std::vector<uint8_t> header_buffer;
                if (!NetworkUtils::recvAll(client->socket_fd, header_buffer, sizeof(MessageHeader))) { ILOG << "Client " << current_client_id << " disconnected (header read failed)."; break; }
                MessageHeader header;
                if (header_buffer.size() >= sizeof(MessageHeader)) {
                    std::memcpy(&header, header_buffer.data(), sizeof(MessageHeader));
                }
                else {
                    throw std::runtime_error("Header buffer size insufficient");
                }
                if (header.magic != MessageHeader::MAGIC_NUMBER) { WLOG << "Invalid magic number from " << current_client_id; break; }
                std::vector<uint8_t> data_buffer;
                if (header.data_length > 0 && !NetworkUtils::recvAll(client->socket_fd, data_buffer, header.data_length)) { ILOG << "Client " << current_client_id << " disconnected (payload read failed)."; break; }
                NetworkMessage message = NetworkMessage::deserialize(header_buffer, std::move(data_buffer));
                client->last_heartbeat = std::chrono::steady_clock::now();
                if (message.header.type == MessageType::HANDSHAKE || message.header.type == MessageType::SLAVE_REGISTER) {
                    std::string new_slave_id(message.data.begin(), message.data.end());
                    if (current_client_id != new_slave_id) {
                        std::lock_guard<std::mutex> lock(clients_mutex_);
                        clients_.erase(current_client_id);
                        client->slave_id = new_slave_id;
                        clients_[new_slave_id] = client;
                        ILOG << "Client ID updated: " << current_client_id << " -> " << new_slave_id;
                        current_client_id = new_slave_id;
                        if (connection_callback_) connection_callback_(current_client_id, true);
                    }
                }
                if (message_callback_) message_callback_(message, current_client_id);
            }
            catch (const std::exception& e) { ELOG << "Exception in client handler for " << current_client_id << ": " << e.what(); break; }
        }
        client->is_active = false;
        disconnectClient(current_client_id);
        ILOG << "Client handler ended for " << current_client_id;
    }

    bool NetworkServer::sendMessage(const std::string& client_id, const NetworkMessage& message) {
        std::shared_ptr<ClientConnection> client;
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            auto it = clients_.find(client_id);
            if (it == clients_.end() || !it->second->is_active) return false;
            client = it->second;
        }
        return NetworkUtils::sendAll(client->socket_fd, message.serialize());
    }

    void NetworkServer::heartbeatThread() {
        while (running_) {
            std::this_thread::sleep_for(std::chrono::seconds(cfg_.heartbeat_timeout_seconds / 2));
            if (!running_) break;
            auto now = std::chrono::steady_clock::now();
            std::vector<std::string> disconnected_clients;
            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                for (auto& [id, client] : clients_) {
                    if (std::chrono::duration_cast<std::chrono::seconds>(now - client->last_heartbeat).count() > cfg_.heartbeat_timeout_seconds) {
                        disconnected_clients.push_back(id);
                    }
                }
            }
            for (const auto& id : disconnected_clients) {
                WLOG << "Client " << id << " timed out. Disconnecting.";
                disconnectClient(id);
            }
        }
    }

    void NetworkServer::disconnectClient(const std::string& client_id) {
        std::shared_ptr<ClientConnection> client;
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            auto it = clients_.find(client_id);
            if (it != clients_.end()) {
                client = it->second;
                clients_.erase(it);
            }
        }
        if (client) {
            client->is_active = false;
#ifdef _WIN32
            shutdown(client->socket_fd, SD_BOTH); closesocket(client->socket_fd);
#else
            shutdown(client->socket_fd, SHUT_RDWR); close(client->socket_fd);
#endif
            if (connection_callback_) connection_callback_(client_id, false);
            ILOG << "Client disconnected: " << client_id;
        }
    }

    std::vector<std::string> NetworkServer::getConnectedClients() const {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        std::vector<std::string> client_ids;
        for (const auto& [id, conn] : clients_) if (conn->is_active) client_ids.push_back(id);
        return client_ids;
    }

    bool NetworkServer::isClientConnected(const std::string& client_id) const {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        auto it = clients_.find(client_id);
        return it != clients_.end() && it->second->is_active;
    }

    size_t NetworkServer::getClientCount() const {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        return clients_.size();
    }

    bool NetworkServer::broadcastMessage(const NetworkMessage& message) {
        std::vector<std::shared_ptr<ClientConnection>> targets;

        /// 1. Acquire the lock and copy only the target list
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (const auto& [id, conn] : clients_) {
                if (conn->is_active) targets.push_back(conn);
            }
        }

        /// 2. Send without holding the lock
        /// (sendAll is assumed to be thread-safe, or per-socket locking is required)
        bool all_success = true;
        auto data = message.serialize(); /// Perform serialization only once

        for (const auto& conn : targets) {
            if (!NetworkUtils::sendAll(conn->socket_fd, data)) {
                all_success = false;
            }
        }
        return all_success;
    }


    std::string NetworkServer::sockaddrToString(const sockaddr_in& addr) {
        char buffer[INET_ADDRSTRLEN];
        if (inet_ntop(AF_INET, &addr.sin_addr, buffer, INET_ADDRSTRLEN)) return std::string(buffer);
        return "?.?.?.?";
    }

    //========================================
    // NetworkClient Implementation
    //========================================
    NetworkClient::NetworkClient() : client_socket_(-1), connected_(false), shutdown_requested_(false) {
#ifdef _WIN32
        WSADATA wsaData; WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
        slave_id_ = NetworkUtils::generateSlaveId();
    }

    NetworkClient::~NetworkClient() {
        disconnect();
#ifdef _WIN32
        WSACleanup();
#endif
    }

    bool NetworkClient::connect(const std::string& address, uint16_t port) {
        if (connected_) return false;
        server_address_ = address;
        server_port_ = port;
        if (!initializeSocket()) return false;
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        inet_pton(AF_INET, address.c_str(), &server_addr.sin_addr);
        if (::connect(client_socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
            ELOG << "Failed to connect to server: " << address << ":" << port;
            cleanupSocket();
            return false;
        }
        connected_ = true;
        shutdown_requested_ = false;
        client_thread_ = std::thread(&NetworkClient::clientThread, this);
        NetworkMessage reg_msg(MessageType::SLAVE_REGISTER, std::vector<uint8_t>(slave_id_.begin(), slave_id_.end()));
        if (!sendMessage(reg_msg)) { disconnect(); return false; }
        heartbeat_thread_ = std::thread(&NetworkClient::heartbeatThread, this);
        if (connection_callback_) connection_callback_(slave_id_, true);
        ILOG << "Connected to server: " << address << ":" << port << " as " << slave_id_;
        return true;
    }

    void NetworkClient::disconnect() {
        if (!connected_) return;
        shutdown_requested_ = true;
        connected_ = false;
        cleanupSocket();
        if (client_thread_.joinable()) client_thread_.join();
        if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
        if (connection_callback_) connection_callback_(slave_id_, false);
        ILOG << "Disconnected from server";
    }

    bool NetworkClient::initializeSocket() {
        client_socket_ = static_cast<int>(socket(AF_INET, SOCK_STREAM, 0));
        return client_socket_ >= 0;
    }

    void NetworkClient::cleanupSocket() {
        if (client_socket_ >= 0) {
#ifdef _WIN32
            shutdown(client_socket_, SD_BOTH); closesocket(client_socket_);
#else
            shutdown(client_socket_, SHUT_RDWR); close(client_socket_);
#endif
            client_socket_ = -1;
        }
    }

    bool NetworkClient::sendMessage(const NetworkMessage& message) {
        if (!connected_) return false;
        return NetworkUtils::sendAll(client_socket_, message.serialize());
    }

    void NetworkClient::clientThread() {
        while (connected_ && !shutdown_requested_) {
            try {
                std::vector<uint8_t> header_buffer;
                if (!NetworkUtils::recvAll(client_socket_, header_buffer, sizeof(MessageHeader))) {
                    if (connected_) ILOG << "Server disconnected (header read failed).";
                    break;
                }
                MessageHeader header;
                if (header_buffer.size() >= sizeof(MessageHeader)) {
                    std::memcpy(&header, header_buffer.data(), sizeof(MessageHeader));
                }
                else {
                    throw std::runtime_error("Header buffer size insufficient");
                }
                if (header.magic != MessageHeader::MAGIC_NUMBER) { WLOG << "Invalid magic number from server."; break; }
                std::vector<uint8_t> data_buffer;
                if (header.data_length > 0 && !NetworkUtils::recvAll(client_socket_, data_buffer, header.data_length)) {
                    if (connected_) ILOG << "Server disconnected (payload read failed).";
                    break;
                }
                NetworkMessage message = NetworkMessage::deserialize(header_buffer, std::move(data_buffer));
                if (message_callback_) message_callback_(message, "server");
            }
            catch (const std::exception& e) {
                if (connected_) ELOG << "Exception in client thread: " << e.what();
                break;
            }
        }
        if (connected_) {
            connected_ = false;
            if (connection_callback_) connection_callback_(slave_id_, false);
        }
    }

    void NetworkClient::heartbeatThread() {
        while (connected_ && !shutdown_requested_) {
            std::this_thread::sleep_for(std::chrono::seconds(heartbeat_interval));
            if (connected_ && !shutdown_requested_) {
                NetworkMessage heartbeat(MessageType::HEARTBEAT, std::vector<uint8_t>(slave_id_.begin(), slave_id_.end()));
                if (!sendMessage(heartbeat)) {
                    WLOG << "Failed to send heartbeat";
                }
            }
        }
    }

} // namespace DPApp