#pragma once

#include <cstdint>
#include <system_error>
#include <windows.h>
#include <BaseTsd.h>   /// ssize_t를 위해
typedef SSIZE_T ssize_t;

namespace pointclouds2 {
    constexpr size_t CHUNK_NUM_PTS = 1000000; // 100K points
    constexpr size_t CHUNK_SIZE_IN_FLOAT = CHUNK_NUM_PTS * 3; /// 점 하나 = float 좌표 3개
    constexpr size_t CHUNK_SIZE_IN_BYTE = CHUNK_SIZE_IN_FLOAT * sizeof(float);

    ssize_t read_at(HANDLE hFile, void* buffer, size_t size, ssize_t offset, std::error_code& ec) {
        OVERLAPPED ov = {};
        ov.Offset = static_cast<DWORD>(offset & 0xFFFFFFFF);
        ov.OffsetHigh = static_cast<DWORD>((offset >> 32) & 0xFFFFFFFF);
        DWORD bytesRead = 0;

        if (!ReadFile(hFile, buffer, static_cast<DWORD>(size), &bytesRead, &ov)) {
            ec = std::error_code(GetLastError(), std::system_category());
            return -1;
        }

        ec.clear();
        return static_cast<ssize_t>(bytesRead);
    }

    bool loadPointclouds2(const std::string& pc2Path,
        std::vector<float>& outPoints) {
        HANDLE hFile = CreateFileA(
            pc2Path.c_str(),
            GENERIC_READ, /// 읽기 모드
            FILE_SHARE_READ, /// 읽기 공유 모드
            NULL, /// 보안 속성은 상속되지 않음
            OPEN_EXISTING, /// 대상 파일이 없으면 실패
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, /// 특별한 속성 없음 | 순차 읽기 최적화
            NULL /// 새로 생성되는 파일에 전달할 속성
        );

        if (hFile == INVALID_HANDLE_VALUE) {
            std::cerr << "Failed to open file: " << pc2Path
                << " (Error: " << GetLastError() << ")" << std::endl;
            return false;
        }

        LARGE_INTEGER fileSize;
        if (!GetFileSizeEx(hFile, &fileSize)) {
            std::cerr << "Failed to get file size (Error: " << GetLastError() << ")" << std::endl;
            CloseHandle(hFile);
            return false;
        }

        if (fileSize.QuadPart < 12) { /// float offset 값 3개
            std::cerr << "File too small: " << fileSize.QuadPart << " bytes" << std::endl;
            CloseHandle(hFile);
            return false;
        }

        size_t pointDataSize = fileSize.QuadPart - 12;

        if (pointDataSize % 12 != 0) {
            std::cerr << "Warning: File size not aligned to point data (12 bytes per point)" << std::endl;
            CloseHandle(hFile);
            return false;
        }

        std::error_code ec;
        std::vector<float> buf(CHUNK_SIZE_IN_FLOAT);

        /// offset 값들
        float offset[3] = { 0.f, 0.f, 0.f };
        auto ret = read_at(hFile, offset, sizeof(float) * 3, 0, ec);

        size_t numPoints = pointDataSize / 12; // float(4바이트) 3개
        std::cout << "Start loading " << numPoints << " points..." << std::endl;

        /// 전체 점을 위한 공간 예약
        try {
            outPoints.reserve(numPoints * 3);
        }
        catch (const std::bad_alloc& e) {
            std::cerr << "Failed to allocate memory for " << numPoints << " points: " << e.what() << std::endl;
            CloseHandle(hFile);
            return false;
        }

        /// 배치 단위로 점 읽기
        std::vector<float> buffer(CHUNK_SIZE_IN_FLOAT);
        size_t currentOffset = 12; /// offset 데이터 다음부터 시작
        size_t remainingBytes = pointDataSize;
        size_t pointsLoaded = 0;

        while (remainingBytes > 0) {
            /// 이번 반복에서 읽을 바이트 수 계산
            size_t bytesToRead = static_cast<size_t>( (std::min)(static_cast<size_t>(CHUNK_SIZE_IN_FLOAT * sizeof(float)), remainingBytes));

            /// 배치 읽기
            ret = read_at(hFile, buffer.data(), bytesToRead, currentOffset, ec);
            if (ret < 0) {
                std::cerr << "Read error at offset " << currentOffset << ": " << ec.message() << std::endl;
                CloseHandle(hFile);
                return false;
            }

            if (ret == 0) {
                /// 예상치 못한 EOF - 파일이 예상보다 짧음
                std::cerr << "Unexpected EOF at offset " << currentOffset
                    << ". Expected " << remainingBytes << " more bytes." << std::endl;
                CloseHandle(hFile);
                return false;
            }

            if (static_cast<size_t>(ret) != bytesToRead) {
                /// 부분 읽기 - 이 문맥에서는 이것도 예상치 못한 상황
                std::cerr << "Warning: Partial read at offset " << currentOffset
                    << ". Requested " << bytesToRead << " bytes, got " << ret << " bytes." << std::endl;
                CloseHandle(hFile);
                return false;
            }

            /// 출력 벡터에 이어붙임
            size_t floatsRead = ret / sizeof(float);
            outPoints.insert(outPoints.end(), buffer.begin(), buffer.begin() + floatsRead);

            pointsLoaded += floatsRead / 3;
            currentOffset += ret;
            remainingBytes -= ret;

            // 약 1000만 점마다 진행 상황 보고
            if (pointsLoaded % CHUNK_NUM_PTS == 0) {
                std::cout << "Loaded " << pointsLoaded << " points..." << std::endl;
            }
        }

        CloseHandle(hFile);

        std::cout << "Successfully loaded " << pointsLoaded << " points from " << pc2Path << std::endl;
        std::cout << "Total floats: " << outPoints.size() << std::endl;

        return true;
    }

    /// 위 오버로드와 동일한 로직이지만, 파일 전체가 아니라 beginPtsIdx번째 점부터
    /// numPtsToRead0개만 읽는다 (부분 로딩 지원).
    bool loadPointclouds2(const std::string& pc2Path,
        std::vector<float>& outPoints,
        const size_t numPtsToRead0,
        const size_t beginPtsIdx = 0) {
        HANDLE hFile = CreateFileA(
            pc2Path.c_str(),
            GENERIC_READ, /// 읽기 모드
            FILE_SHARE_READ, /// 읽기 공유 모드
            NULL, /// 보안 속성은 상속되지 않음
            OPEN_EXISTING, /// 대상 파일이 없으면 실패
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, /// 특별한 속성 없음 | 순차 읽기 최적화
            NULL /// 새로 생성되는 파일에 전달할 속성
        );

        if (hFile == INVALID_HANDLE_VALUE) {
            std::cerr << "Failed to open file: " << pc2Path
                << " (Error: " << GetLastError() << ")" << std::endl;
            return false;
        }

        LARGE_INTEGER fileSize;
        if (!GetFileSizeEx(hFile, &fileSize)) {
            std::cerr << "Failed to get file size (Error: " << GetLastError() << ")" << std::endl;
            CloseHandle(hFile);
            return false;
        }

        size_t pointDataSize = fileSize.QuadPart - 12;

        if (fileSize.QuadPart < 12) { /// float offset 값 3개
            std::cerr << "File too small: " << fileSize.QuadPart << " bytes" << std::endl;
            CloseHandle(hFile);
            return false;
        }

        size_t numPoints = pointDataSize / 12; // float(4바이트) 3개

        if (pointDataSize % 12 != 0) {
            std::cerr << "Warning: File size not aligned to point data (12 bytes per point)" << std::endl;
            CloseHandle(hFile);
            return false;
        }

        size_t numPtsToRead;
        if (numPoints < (beginPtsIdx + numPtsToRead0)) {

            numPtsToRead = numPoints - beginPtsIdx;
            std::cerr << "It is not possible to read " << numPtsToRead0 << " points from " << beginPtsIdx << " th point.";
            std::cerr << " Instead, it can read " << numPtsToRead << " points\n";
            return false;
        }
        else {
            numPtsToRead = numPtsToRead0;
        }

        std::error_code ec;
        std::vector<float> buf(CHUNK_SIZE_IN_FLOAT);

        /// offset 값들
        float offset[3] = { 0.f, 0.f, 0.f };
        auto ret = read_at(hFile, offset, sizeof(float) * 3, 0, ec);

        std::cout << "Start loading " << numPtsToRead << " points..." << std::endl;

        /// 전체 점을 위한 공간 예약
        try {
            outPoints.reserve(numPtsToRead * 3);
        }
        catch (const std::bad_alloc& e) {
            std::cerr << "Failed to allocate memory for " << numPtsToRead << " points: " << e.what() << std::endl;
            CloseHandle(hFile);
            return false;
        }

        /// 배치 단위로 점 읽기
        size_t buffer_size = static_cast<size_t>((std::min)(static_cast<size_t>(numPtsToRead * 3 * sizeof(float)), CHUNK_SIZE_IN_FLOAT));
        std::vector<float> buffer(buffer_size);

        size_t currentOffset = 12 + (beginPtsIdx * 3 * sizeof(float)); /// 읽기 시작 위치
        size_t ReamingNumPts = numPtsToRead;
        size_t pointsLoaded = 0;

        while (ReamingNumPts > 0) {
            /// 이번 반복에서 읽을 바이트 수 계산
            size_t bytesToRead = static_cast<size_t>(
                (std::min) (
                    static_cast<size_t>(buffer.size() * sizeof(float)),
                    ReamingNumPts * 3 *sizeof(float)
                    )
                );

            /// 배치 읽기
            ret = read_at(hFile, buffer.data(), bytesToRead, currentOffset, ec);
            if (ret < 0) {
                std::cerr << "Read error at offset " << currentOffset << ": " << ec.message() << std::endl;
                CloseHandle(hFile);
                return false;
            }

            if (ret == 0) {
                /// 예상치 못한 EOF - 파일이 예상보다 짧음
                std::cerr << "Unexpected EOF at offset " << currentOffset
                    << ". Expected " << (ReamingNumPts * 3 * sizeof(float)) << " more bytes." << std::endl;
                CloseHandle(hFile);
                return false;
            }

            if (static_cast<size_t>(ret) != bytesToRead) {
                /// 부분 읽기 - 이 문맥에서는 이것도 예상치 못한 상황
                std::cerr << "Warning: Partial read at offset " << currentOffset
                    << ". Requested " << bytesToRead << " bytes, got " << ret << " bytes." << std::endl;
                CloseHandle(hFile);
                return false;
            }

            /// 출력 벡터에 이어붙임
            size_t floatsRead = ret / sizeof(float);
            outPoints.insert(outPoints.end(), buffer.begin(), buffer.begin() + floatsRead);

            pointsLoaded += floatsRead / 3;
            currentOffset += ret;
            ReamingNumPts -= floatsRead / 3;

            // 진행 상황 보고
            if (pointsLoaded % CHUNK_NUM_PTS == 0) {
                std::cout << "Loaded " << pointsLoaded << " points..." << std::endl;
            }
        }

        CloseHandle(hFile);

        std::cout << "Successfully loaded " << pointsLoaded << " points from " << pc2Path << std::endl;
        std::cout << "Total floats: " << outPoints.size() << std::endl;

        return true;
    }
}
