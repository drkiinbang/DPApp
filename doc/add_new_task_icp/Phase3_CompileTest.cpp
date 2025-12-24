/// ============================================================================
/// Phase 3 컴파일 및 직렬화 테스트 코드
/// ============================================================================
/// 
/// 직렬화/역직렬화가 정확히 동작하는지 확인합니다.
/// ============================================================================

#include "../include/IcpTypes.h"
#include "../include/bimtree/IcpSerialization.hpp"

#include <iostream>
#include <cmath>

/// 부동소수점 비교 헬퍼
inline bool floatEquals(float a, float b, float epsilon = 1e-6f) {
    return std::abs(a - b) < epsilon;
}

/// Phase 3 직렬화 테스트
void testIcpSerializationCompile() {
    std::cout << "=== Phase 3 Serialization Test ===" << std::endl;
    
    bool allPassed = true;

    /// 1. Transform4x4 직렬화 테스트
    {
        icp::Transform4x4 original = icp::Transform4x4::translation(1.5f, 2.5f, 3.5f);
        
        std::vector<uint8_t> buffer;
        icp::serializeTransform(original, buffer);
        
        size_t offset = 0;
        icp::Transform4x4 restored = icp::deserializeTransform(buffer.data(), offset);
        
        float tx1, ty1, tz1, tx2, ty2, tz2;
        original.getTranslation(tx1, ty1, tz1);
        restored.getTranslation(tx2, ty2, tz2);
        
        bool match = floatEquals(tx1, tx2) && floatEquals(ty1, ty2) && floatEquals(tz1, tz2);
        std::cout << "Transform4x4 serialization: " << (match ? "PASS" : "FAIL") << std::endl;
        allPassed &= match;
    }

    /// 2. IcpConfig 직렬화 테스트
    {
        icp::IcpConfig original;
        original.maxIterations = 100;
        original.convergenceThreshold = 1e-7f;
        original.maxCorrespondenceDistance = 2.0f;
        original.downsampleRatio = 5;
        original.useNormalFiltering = true;
        original.verbose = true;
        
        std::vector<uint8_t> buffer;
        icp::serializeConfig(original, buffer);
        
        size_t offset = 0;
        icp::IcpConfig restored = icp::deserializeConfig(buffer.data(), offset);
        
        bool match = (original.maxIterations == restored.maxIterations) &&
                     (original.downsampleRatio == restored.downsampleRatio) &&
                     floatEquals(original.convergenceThreshold, restored.convergenceThreshold) &&
                     floatEquals(original.maxCorrespondenceDistance, restored.maxCorrespondenceDistance) &&
                     (original.useNormalFiltering == restored.useNormalFiltering) &&
                     (original.verbose == restored.verbose);
        
        std::cout << "IcpConfig serialization: " << (match ? "PASS" : "FAIL") << std::endl;
        allPassed &= match;
    }

    /// 3. IcpChunk 직렬화 테스트
    {
        icp::IcpChunk original;
        original.chunk_id = 42;
        original.sourcePoints = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
        original.targetPoints = {{7.0f, 8.0f, 9.0f}, {10.0f, 11.0f, 12.0f}, {13.0f, 14.0f, 15.0f}};
        original.targetNormals = {{0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}};
        original.initialTransform = icp::Transform4x4::translation(0.1f, 0.2f, 0.3f);
        original.config.maxIterations = 75;
        original.source_min_x = -1.0f; original.source_max_x = 10.0f;
        original.source_min_y = -2.0f; original.source_max_y = 20.0f;
        original.source_min_z = -3.0f; original.source_max_z = 30.0f;
        
        std::vector<uint8_t> buffer = icp::serializeIcpChunk(original);
        icp::IcpChunk restored = icp::deserializeIcpChunk(buffer);
        
        bool match = (original.chunk_id == restored.chunk_id) &&
                     (original.sourcePoints.size() == restored.sourcePoints.size()) &&
                     (original.targetPoints.size() == restored.targetPoints.size()) &&
                     (original.targetNormals.size() == restored.targetNormals.size()) &&
                     (original.config.maxIterations == restored.config.maxIterations) &&
                     floatEquals(original.source_min_x, restored.source_min_x) &&
                     floatEquals(original.source_max_x, restored.source_max_x);
        
        /// 포인트 데이터 확인
        if (match && original.sourcePoints.size() > 0) {
            match = floatEquals(original.sourcePoints[0][0], restored.sourcePoints[0][0]) &&
                    floatEquals(original.sourcePoints[0][1], restored.sourcePoints[0][1]) &&
                    floatEquals(original.sourcePoints[0][2], restored.sourcePoints[0][2]);
        }
        
        std::cout << "IcpChunk serialization: " << (match ? "PASS" : "FAIL") 
                  << " (buffer size: " << buffer.size() << " bytes)" << std::endl;
        allPassed &= match;
    }

    /// 4. IcpResult 직렬화 테스트
    {
        icp::IcpResult original;
        original.chunk_id = 123;
        original.transform = icp::Transform4x4::translation(5.0f, 6.0f, 7.0f);
        original.localTransform = icp::Transform4x4::translation(0.5f, 0.6f, 0.7f);
        original.finalRMSE = 0.0123f;
        original.initialRMSE = 0.5678f;
        original.actualIterations = 25;
        original.correspondenceCount = 1000;
        original.sourcePointCount = 5000;
        original.targetPointCount = 3000;
        original.converged = true;
        original.success = true;
        original.processingTimeMs = 123.456;
        original.errorMessage = "";  // 빈 메시지
        
        std::vector<uint8_t> buffer = icp::serializeIcpResult(original);
        icp::IcpResult restored = icp::deserializeIcpResult(buffer);
        
        bool match = (original.chunk_id == restored.chunk_id) &&
                     floatEquals(original.finalRMSE, restored.finalRMSE) &&
                     floatEquals(original.initialRMSE, restored.initialRMSE) &&
                     (original.actualIterations == restored.actualIterations) &&
                     (original.correspondenceCount == restored.correspondenceCount) &&
                     (original.sourcePointCount == restored.sourcePointCount) &&
                     (original.targetPointCount == restored.targetPointCount) &&
                     (original.converged == restored.converged) &&
                     (original.success == restored.success) &&
                     (std::abs(original.processingTimeMs - restored.processingTimeMs) < 0.001);
        
        std::cout << "IcpResult serialization (success): " << (match ? "PASS" : "FAIL")
                  << " (buffer size: " << buffer.size() << " bytes)" << std::endl;
        allPassed &= match;
    }

    /// 5. IcpResult with error message 직렬화 테스트
    {
        icp::IcpResult original;
        original.chunk_id = 999;
        original.success = false;
        original.converged = false;
        original.errorMessage = "Test error message: something went wrong!";
        
        std::vector<uint8_t> buffer = icp::serializeIcpResult(original);
        icp::IcpResult restored = icp::deserializeIcpResult(buffer);
        
        bool match = (original.chunk_id == restored.chunk_id) &&
                     (original.success == restored.success) &&
                     (original.converged == restored.converged) &&
                     (original.errorMessage == restored.errorMessage);
        
        std::cout << "IcpResult serialization (error): " << (match ? "PASS" : "FAIL")
                  << " (message: \"" << restored.errorMessage << "\")" << std::endl;
        allPassed &= match;
    }

    /// 6. NetworkUtils wrapper 테스트
    {
        icp::IcpChunk chunk;
        chunk.chunk_id = 777;
        chunk.sourcePoints = {{1.0f, 2.0f, 3.0f}};
        chunk.targetPoints = {{4.0f, 5.0f, 6.0f}};
        
        auto buffer = DPApp::NetworkUtils::serializeIcpChunk(chunk);
        auto restored = DPApp::NetworkUtils::deserializeIcpChunk(buffer);
        
        bool match = (chunk.chunk_id == restored.chunk_id);
        std::cout << "NetworkUtils wrapper: " << (match ? "PASS" : "FAIL") << std::endl;
        allPassed &= match;
    }

    std::cout << "\n=== Phase 3 Test Result: " << (allPassed ? "ALL PASSED" : "SOME FAILED") 
              << " ===" << std::endl;
}

/// ============================================================================
/// 사용 방법
/// ============================================================================
/// 
/// 1. IcpSerialization.hpp를 include/bimtree/ 폴더에 복사
/// 
/// 2. MasterApplication.cpp에 include 추가:
///    #include "../include/bimtree/IcpSerialization.hpp"
/// 
/// 3. main() 함수에서 테스트 호출:
///    testIcpSerializationCompile();
///    return 0;
/// 
/// 4. 빌드 및 실행
/// 
/// 5. 모든 테스트가 PASS되면 Phase 3 완료
/// ============================================================================
