// LasTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "LasLibWrapper.hpp"

int main() {

    //const std::string path = "F:\\repository\\DPApp\\data\\samsung_test\\samsung.las";
    const std::string path = "F:\\repository\\DPApp\\data\\laz1.4\\USGS_LPC_AK_SouthEastLandslides_D22_886504.laz";

    las::LASToolsReader reader;

    if (reader.open(path)) {
        // 헤더 정보 출력
        reader.printHeader();

        /// 조건부 로딩 - 지면 분류만
        //reader.loadFilteredPoints([](const auto& point) {
        //    return point.classification == 2; // Ground only
        //    });

        /// 공간 쿼리
        //auto subset = reader.getPointsInBounds(100.0, 200.0, 150.0, 250.0);
        //std::cout << "Points in area: " << subset.size() << '\n';

        /// CSV 내보내기
        //reader.exportToCSV("filtered.csv", las::ExportFormat::XYZIC);

        /// 통계 생성
        //reader.generateClassificationStats();

        /// 포인트 간 거리 계산
        //if (reader.getLoadedPointCount() >= 2) {
        //    const auto& points = reader.getLoadedPoints();
        //    double dist = points[0].distance3D(points[1]);
        //    std::cout << "Distance between first two points: " << dist << "m\n";
        //}

        if(!reader.loadPointRange(0, 10))
            return 1;

        auto loadedPoints = reader.getLoadedPoints();
        for (const auto& p : loadedPoints) {
            std::cout << p.x << "\t";
            std::cout << p.y << "\t";
            std::cout << p.z << "\n";
        }

        std::cout << "----------------------------------------------------\n";

        if (!reader.loadPointRange(5, 10))
            return 1;

        loadedPoints = reader.getLoadedPoints();
        for (const auto& p : loadedPoints) {
            std::cout << p.x << "\t";
            std::cout << p.y << "\t";
            std::cout << p.z << "\n";
        }

    }

    return 0;
}