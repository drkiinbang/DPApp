// LasTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "LasLibWrapper.hpp"

int main() {

    //const std::string path = "F:\\repository\\DPApp\\data\\samsung_test\\samsung.las";
    //const std::string path = "F:\\repository\\DPApp\\LasTest\\sample.las";
    const std::string path = "F:\\repository\\DPApp\\data\\laz1.4\\USGS_LPC_AK_SouthEastLandslides_D22_886504.las";

    las::LASToolsReader reader;

    if (reader.open(path)) {
        // ��� ���� ���
        reader.printHeader();

        // ���Ǻ� �ε� - ���� �з���
        reader.loadFilteredPoints([](const auto& point) {
            return point.classification == 2; // Ground only
            });

        // ���� ����
        auto subset = reader.getPointsInBounds(100.0, 200.0, 150.0, 250.0);
        std::cout << "Points in area: " << subset.size() << '\n';

        // CSV ��������
        reader.exportToCSV("filtered.csv", las::ExportFormat::XYZIC);

        // ��� ����
        reader.generateClassificationStats();

        // ����Ʈ �� �Ÿ� ���
        if (reader.getLoadedPointCount() >= 2) {
            const auto& points = reader.getLoadedPoints();
            double dist = points[0].distance3D(points[1]);
            std::cout << "Distance between first two points: " << dist << "m\n";
        }

        if(!reader.loadPointRange(10, 19))
            return 1;

        auto& loadedPoints = reader.getLoadedPoints();
        for (const auto& p : loadedPoints) {
        }

    }

    return 0;
}