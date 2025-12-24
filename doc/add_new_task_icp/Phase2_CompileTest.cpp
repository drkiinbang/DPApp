/// ============================================================================
/// Phase 2 컴파일 테스트 코드
/// ============================================================================
/// 
/// 이 코드를 임시로 추가하여 IcpCore.hpp가 정상 컴파일되는지 확인합니다.
/// 테스트 후 삭제하면 됩니다.
/// ============================================================================

#include "../include/IcpTypes.h"
#include "../include/bimtree/IcpCore.hpp"

#include <iostream>
#include <atomic>

/// Phase 2 컴파일 및 기능 테스트
void testIcpCoreCompile() {
    std::cout << "=== Phase 2 IcpCore Test ===" << std::endl;

    /// 1. 테스트 데이터 생성
    std::vector<std::array<float, 3>> sourcePoints = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        {1.0f, 1.0f, 0.0f},
        {1.0f, 0.0f, 1.0f},
        {0.0f, 1.0f, 1.0f},
        {1.0f, 1.0f, 1.0f}
    };

    /// Target = Source + (0.1, 0.2, 0.3) translation
    std::vector<std::array<float, 3>> targetPoints;
    for (const auto& p : sourcePoints) {
        targetPoints.push_back({p[0] + 0.1f, p[1] + 0.2f, p[2] + 0.3f});
    }

    /// 2. KD-Tree 테스트
    icp::PointCloudAdaptor adaptor(targetPoints);
    icp::KdTree3D tree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();
    std::cout << "KD-Tree built successfully" << std::endl;

    /// 3. Correspondence 찾기 테스트
    std::vector<icp::Correspondence> correspondences;
    icp::findCorrespondences(sourcePoints, tree, 1.0f, correspondences);
    std::cout << "Found " << correspondences.size() << " correspondences" << std::endl;

    /// 4. SVD 변환 추정 테스트
    icp::Transform4x4 transform = icp::estimateRigidTransformSVD(
        sourcePoints, targetPoints, correspondences);
    
    float tx, ty, tz;
    transform.getTranslation(tx, ty, tz);
    std::cout << "Estimated translation: (" << tx << ", " << ty << ", " << tz << ")" << std::endl;
    std::cout << "Expected translation: (0.1, 0.2, 0.3)" << std::endl;

    /// 5. RMSE 계산 테스트
    float rmse = icp::computeRMSE(correspondences);
    std::cout << "Initial RMSE: " << rmse << std::endl;

    /// 6. 다운샘플링 테스트
    auto downsampled = icp::downsampleUniform(sourcePoints, 2);
    std::cout << "Downsampled: " << sourcePoints.size() << " -> " << downsampled.size() << std::endl;

    /// 7. 전체 ICP 테스트
    icp::IcpChunk chunk;
    chunk.chunk_id = 1;
    chunk.sourcePoints = sourcePoints;
    chunk.targetPoints = targetPoints;
    chunk.initialTransform = icp::Transform4x4::identity();
    chunk.config.maxIterations = 50;
    chunk.config.convergenceThreshold = 1e-6f;
    chunk.config.maxCorrespondenceDistance = 1.0f;
    chunk.config.minCorrespondences = 3;
    chunk.config.useNormalFiltering = false;

    std::atomic<bool> cancelRequested{false};
    icp::IcpResult result = icp::runIcp(chunk, cancelRequested);

    std::cout << "\n=== ICP Result ===" << std::endl;
    std::cout << "Success: " << (result.success ? "Yes" : "No") << std::endl;
    std::cout << "Converged: " << (result.converged ? "Yes" : "No") << std::endl;
    std::cout << "Iterations: " << result.actualIterations << std::endl;
    std::cout << "Initial RMSE: " << result.initialRMSE << std::endl;
    std::cout << "Final RMSE: " << result.finalRMSE << std::endl;
    std::cout << "Processing time: " << result.processingTimeMs << " ms" << std::endl;

    result.transform.getTranslation(tx, ty, tz);
    std::cout << "Final translation: (" << tx << ", " << ty << ", " << tz << ")" << std::endl;

    /// 8. 결과 집계 테스트
    std::vector<icp::IcpResult> results = {result};
    icp::Transform4x4 aggregated = icp::aggregateResultsWeighted(results);
    const icp::IcpResult* best = icp::selectBestResult(results);
    
    std::cout << "\nAggregation test: " << (best != nullptr ? "OK" : "Failed") << std::endl;
    std::cout << "=== Phase 2 Test Complete ===" << std::endl;

    (void)aggregated;  // 경고 방지
}

/// ============================================================================
/// 사용 방법
/// ============================================================================
/// 
/// 1. IcpCore.hpp를 F:\repository\DPApp\include\bimtree\ 폴더에 복사
/// 
/// 2. MasterApplication.cpp 상단에 include 추가:
///    #include "../include/bimtree/IcpCore.hpp"
/// 
/// 3. main() 함수 시작 부분에 테스트 함수 호출:
///    testIcpCoreCompile();
///    return 0;  // 테스트만 실행하고 종료
/// 
/// 4. MasterApp 프로젝트 빌드 및 실행
/// 
/// 5. 콘솔에서 ICP 테스트 결과 확인
/// 
/// 6. 테스트 성공 후 테스트 코드 삭제
/// ============================================================================
