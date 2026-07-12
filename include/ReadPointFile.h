#pragma once

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

namespace ptsfile {
    inline void ltrim(std::string& s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            [](unsigned char ch) { return !std::isspace(ch); }));
    }

    inline void rtrim(std::string& s) {
        s.erase(std::find_if(s.rbegin(), s.rend(),
            [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
    }

    inline void trim(std::string& s) { ltrim(s); rtrim(s); }

    inline size_t countPointsInFile(const std::string& filename, const char skipChar = '\n') {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::clog << "Cannot open the file: " + filename << std::endl;
            return size_t(0);
        }

        std::string firstLine;
        if (!std::getline(file, firstLine)) {
            std::clog << "Empty file: " << filename << std::endl;
            return size_t(0);
        }

        trim(firstLine);

        std::istringstream iss(firstLine);
        size_t pointCount = 0;
        if (iss >> pointCount && iss.eof()) {
            /// 첫 줄이 전체 줄(점) 개수를 담고 있는 경우
            return pointCount;
        }

        /// 모든 줄을 직접 세어야 하는 경우
        pointCount = 1; /// 첫 줄은 이미 포함되어 있음.
        std::string line;
        while (std::getline(file, line)) {
            trim(line);
            if (!line.empty() && line[0] != skipChar)
                ++pointCount;
        }

        return pointCount;
    }
}