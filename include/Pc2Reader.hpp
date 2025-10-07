#pragma once

#include <cstdint>
#include <system_error>
#include <windows.h>
#include <BaseTsd.h>   /// for ssize_t
typedef SSIZE_T ssize_t;

namespace pc2 {
    constexpr size_t CHUNK_NUM_PTS = 1000000; // 100K points
    constexpr size_t CHUNK_SIZE_IN_FLOAT = CHUNK_NUM_PTS * 3; /// single pt: 3 float coordinates
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
            GENERIC_READ, /// read mode
            FILE_SHARE_READ, /// share read mode
            NULL, /// Security attributes are not inherited
            OPEN_EXISTING, /// If the target file does not exist, then failed
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, /// no specific attribute | sequential read optimization
            NULL /// Attributes passed to a new created file
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

        if (fileSize.QuadPart < 12) { /// 3 float offset values
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

        /// offset values
        float offset[3] = { 0.f, 0.f, 0.f };
        auto ret = read_at(hFile, offset, sizeof(float) * 3, 0, ec);

        size_t numPoints = pointDataSize / 12; // 3 floats(4 bytes)
        std::cout << "Start loading " << numPoints << " points..." << std::endl;

        /// Reserve space for all points
        try {
            outPoints.reserve(numPoints * 3);
        }
        catch (const std::bad_alloc& e) {
            std::cerr << "Failed to allocate memory for " << numPoints << " points: " << e.what() << std::endl;
            CloseHandle(hFile);
            return false;
        }

        /// Read points in batches
        std::vector<float> buffer(CHUNK_SIZE_IN_FLOAT);
        size_t currentOffset = 12; /// Start after offset data
        size_t remainingBytes = pointDataSize;
        size_t pointsLoaded = 0;

        while (remainingBytes > 0) {
            /// Calculate bytes to read in this iteration
            size_t bytesToRead = static_cast<size_t>( (std::min)(static_cast<size_t>(CHUNK_SIZE_IN_FLOAT * sizeof(float)), remainingBytes));

            /// Read batch
            ret = read_at(hFile, buffer.data(), bytesToRead, currentOffset, ec);
            if (ret < 0) {
                std::cerr << "Read error at offset " << currentOffset << ": " << ec.message() << std::endl;
                CloseHandle(hFile);
                return false;
            }

            if (ret == 0) {
                /// Unexpected EOF - file is shorter than expected
                std::cerr << "Unexpected EOF at offset " << currentOffset
                    << ". Expected " << remainingBytes << " more bytes." << std::endl;
                CloseHandle(hFile);
                return false;
            }

            if (static_cast<size_t>(ret) != bytesToRead) {
                /// Partial read - also unexpected in this context
                std::cerr << "Warning: Partial read at offset " << currentOffset
                    << ". Requested " << bytesToRead << " bytes, got " << ret << " bytes." << std::endl;
                CloseHandle(hFile);
                return false;
            }

            /// Append to output vector
            size_t floatsRead = ret / sizeof(float);
            outPoints.insert(outPoints.end(), buffer.begin(), buffer.begin() + floatsRead);

            pointsLoaded += floatsRead / 3;
            currentOffset += ret;
            remainingBytes -= ret;

            // Progress report every ~10M points
            if (pointsLoaded % CHUNK_NUM_PTS == 0) {
                std::cout << "Loaded " << pointsLoaded << " points..." << std::endl;
            }
        }

        CloseHandle(hFile);

        std::cout << "Successfully loaded " << pointsLoaded << " points from " << pc2Path << std::endl;
        std::cout << "Total floats: " << outPoints.size() << std::endl;

        return true;
    }

    bool loadPointclouds2(const std::string& pc2Path,
        std::vector<float>& outPoints,
        const size_t numPtsToRead0,
        const size_t beginPtsIdx = 0) {
        HANDLE hFile = CreateFileA(
            pc2Path.c_str(),
            GENERIC_READ, /// read mode
            FILE_SHARE_READ, /// share read mode
            NULL, /// Security attributes are not inherited
            OPEN_EXISTING, /// If the target file does not exist, then failed
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, /// no specific attribute | sequential read optimization
            NULL /// Attributes passed to a new created file
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
        
        if (fileSize.QuadPart < 12) { /// 3 float offset values
            std::cerr << "File too small: " << fileSize.QuadPart << " bytes" << std::endl;
            CloseHandle(hFile);
            return false;
        }

        size_t numPoints = pointDataSize / 12; // 3 floats(4 bytes)

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

        /// offset values
        float offset[3] = { 0.f, 0.f, 0.f };
        auto ret = read_at(hFile, offset, sizeof(float) * 3, 0, ec);
        
        std::cout << "Start loading " << numPtsToRead << " points..." << std::endl;

        /// Reserve space for all points
        try {
            outPoints.reserve(numPtsToRead * 3);
        }
        catch (const std::bad_alloc& e) {
            std::cerr << "Failed to allocate memory for " << numPtsToRead << " points: " << e.what() << std::endl;
            CloseHandle(hFile);
            return false;
        }

        /// Read points in batches
        size_t buffer_size = static_cast<size_t>((std::min)(static_cast<size_t>(numPtsToRead * 3 * sizeof(float)), CHUNK_SIZE_IN_FLOAT));
        std::vector<float> buffer(buffer_size);

        size_t currentOffset = 12 + (beginPtsIdx * 3 * sizeof(float)); /// Start pos to read
        size_t ReamingNumPts = numPtsToRead;
        size_t pointsLoaded = 0;

        while (ReamingNumPts > 0) {
            /// Calculate bytes to read in this iteration
            size_t bytesToRead = static_cast<size_t>(
                (std::min) (
                    static_cast<size_t>(buffer.size() * sizeof(float)), 
                    ReamingNumPts * 3 *sizeof(float)
                    )
                );

            /// Read batch
            ret = read_at(hFile, buffer.data(), bytesToRead, currentOffset, ec);
            if (ret < 0) {
                std::cerr << "Read error at offset " << currentOffset << ": " << ec.message() << std::endl;
                CloseHandle(hFile);
                return false;
            }

            if (ret == 0) {
                /// Unexpected EOF - file is shorter than expected
                std::cerr << "Unexpected EOF at offset " << currentOffset
                    << ". Expected " << (ReamingNumPts * 3 * sizeof(float)) << " more bytes." << std::endl;
                CloseHandle(hFile);
                return false;
            }

            if (static_cast<size_t>(ret) != bytesToRead) {
                /// Partial read - also unexpected in this context
                std::cerr << "Warning: Partial read at offset " << currentOffset
                    << ". Requested " << bytesToRead << " bytes, got " << ret << " bytes." << std::endl;
                CloseHandle(hFile);
                return false;
            }

            /// Append to output vector
            size_t floatsRead = ret / sizeof(float);
            outPoints.insert(outPoints.end(), buffer.begin(), buffer.begin() + floatsRead);

            pointsLoaded += floatsRead / 3;
            currentOffset += ret;
            ReamingNumPts -= floatsRead / 3;

            // Progress report
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