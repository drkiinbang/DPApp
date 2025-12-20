#include <stdexcept>
#include <vector>
#include <string>
#include <cstring> // for memcpy

namespace DPApp {
    namespace NetworkUtils {

        /// ================================================================
        /// Helper Class: BinaryWriter (Safe Serialization)
        /// ================================================================
        class BinaryWriter {
        public:
            std::vector<uint8_t>& buffer;

            explicit BinaryWriter(std::vector<uint8_t>& buf) : buffer(buf) {}

            /// Write primitive types (int, float, double, etc.)
            template <typename T>
            void write(const T& value) {
                const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&value);
                buffer.insert(buffer.end(), ptr, ptr + sizeof(T));
            }

            /// Write a string (length + raw data)
            void writeString(const std::string& str) {
                uint32_t len = static_cast<uint32_t>(str.size());
                write(len);
                if (len > 0) {
                    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(str.data());
                    buffer.insert(buffer.end(), ptr, ptr + len);
                }
            }

            /// Write array or vector data as raw bytes
            void writeBytes(const void* data, size_t size) {
                const uint8_t* ptr = reinterpret_cast<const uint8_t*>(data);
                buffer.insert(buffer.end(), ptr, ptr + size);
            }
        };

        /// ================================================================
        /// Helper Class: BinaryReader (Safe Deserialization with Bounds Check)
        /// ================================================================
        class BinaryReader {
        public:
            const std::vector<uint8_t>& buffer;
            size_t offset;

            explicit BinaryReader(const std::vector<uint8_t>& buf) : buffer(buf), offset(0) {}

            /// Check remaining buffer size
            void checkBounds(size_t size) const {
                if (offset + size > buffer.size()) {
                    throw std::runtime_error("Buffer overflow: insufficient data to read.");
                }
            }

            /// Read primitive types
            template <typename T>
            T read() {
                checkBounds(sizeof(T));
                T value;
                std::memcpy(&value, buffer.data() + offset, sizeof(T));
                offset += sizeof(T);
                return value;
            }

            /// Read a string
            std::string readString() {
                uint32_t len = read<uint32_t>();
                if (len == 0) return "";

                checkBounds(len);
                std::string str(reinterpret_cast<const char*>(buffer.data() + offset), len);
                offset += len;
                return str;
            }

            /// Read a fixed-size block and return a pointer reference
            /// (Use with caution: no copy is performed)
            const void* readBytesRef(size_t size) {
                checkBounds(size);
                const void* ptr = buffer.data() + offset;
                offset += size;
                return ptr;
            }
        };

    } // namespace NetworkUtils
} // namespace DPApp
