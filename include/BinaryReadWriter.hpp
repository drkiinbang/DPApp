#include <stdexcept>
#include <vector>
#include <string>
#include <cstring> // for memcpy

namespace DPApp {
    namespace NetworkUtils {

        /// ================================================================
        /// 헬퍼 클래스: BinaryWriter (안전한 직렬화)
        /// ================================================================
        class BinaryWriter {
        public:
            std::vector<uint8_t>& buffer;

            explicit BinaryWriter(std::vector<uint8_t>& buf) : buffer(buf) {}

            /// 기본형(int, float, double 등) 쓰기
            template <typename T>
            void write(const T& value) {
                const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&value);
                buffer.insert(buffer.end(), ptr, ptr + sizeof(T));
            }

            /// 문자열 쓰기 (길이 + 원본 데이터)
            void writeString(const std::string& str) {
                uint32_t len = static_cast<uint32_t>(str.size());
                write(len);
                if (len > 0) {
                    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(str.data());
                    buffer.insert(buffer.end(), ptr, ptr + len);
                }
            }

            /// 배열/벡터 데이터를 원시 바이트로 쓰기
            void writeBytes(const void* data, size_t size) {
                const uint8_t* ptr = reinterpret_cast<const uint8_t*>(data);
                buffer.insert(buffer.end(), ptr, ptr + size);
            }
        };

        /// ================================================================
        /// 헬퍼 클래스: BinaryReader (범위 검사가 있는 안전한 역직렬화)
        /// ================================================================
        class BinaryReader {
        public:
            const std::vector<uint8_t>& buffer;
            size_t offset;

            explicit BinaryReader(const std::vector<uint8_t>& buf) : buffer(buf), offset(0) {}

            /// 남은 버퍼 크기 확인
            void checkBounds(size_t size) const {
                if (offset + size > buffer.size()) {
                    throw std::runtime_error("Buffer overflow: insufficient data to read.");
                }
            }

            /// 기본형 읽기
            template <typename T>
            T read() {
                checkBounds(sizeof(T));
                T value;
                std::memcpy(&value, buffer.data() + offset, sizeof(T));
                offset += sizeof(T);
                return value;
            }

            /// 문자열 읽기
            std::string readString() {
                uint32_t len = read<uint32_t>();
                if (len == 0) return "";

                checkBounds(len);
                std::string str(reinterpret_cast<const char*>(buffer.data() + offset), len);
                offset += len;
                return str;
            }

            /// 고정 크기 블록을 읽고 포인터 참조를 반환
            /// (주의해서 사용할 것: 복사가 일어나지 않음)
            const void* readBytesRef(size_t size) {
                checkBounds(size);
                const void* ptr = buffer.data() + offset;
                offset += size;
                return ptr;
            }
        };

    } // namespace NetworkUtils
} // namespace DPApp
