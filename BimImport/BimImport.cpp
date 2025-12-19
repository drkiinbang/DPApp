#include "BimImport.h"

#include <filesystem>
namespace fs = std::filesystem;

#include "../include/bim/gltf_mesh.h"

bool find_gltf_files_in_folder(
    const std::string& folder_path,
    std::vector<std::string>& out_gltf_files)
{
    fs::path input_path(folder_path);

    out_gltf_files.clear();

    if (!fs::exists(input_path)) {
        std::cerr << "Input folder '" << folder_path << "' does not exist.\n";
        return false;
    }

    /// Traverse files in the folder
    for (const auto& entry : fs::directory_iterator(input_path)) {
        if (!entry.is_regular_file()) {
            continue;
        }

        const fs::path& p = entry.path();
        const fs::path ext = p.extension();

        /// ext: .gltf / .glb
        if (ext == ".gltf" || ext == ".glb") {
            // 필요 시 UTF-16 → UTF-8 변환 사용 (기존 코드 스타일)
            // std::string utf8FileName = util::wstring_to_utf8(p.wstring());
            // out_gltf_files.push_back(utf8FileName);

            // 간단히 path.string() 사용 (UTF-8 환경 기준)
            out_gltf_files.push_back(p.string());
        }
    }

    return true;
}

namespace util {
    std::string wstring_to_utf8(const std::wstring& wstr) {
        int size_needed = WideCharToMultiByte(CP_UTF8, 0, wstr.data(), (int)wstr.size(), NULL, 0, NULL, NULL);
        std::string utf8str(size_needed, 0);
        WideCharToMultiByte(CP_UTF8, 0, wstr.data(), (int)wstr.size(), &utf8str[0], size_needed, NULL, NULL);
        return utf8str;
    }
}

std::vector<std::string> collect_gltf_files(const std::string& folder_path) {
    std::vector<std::string> gltf_file_paths;
    fs::path input_path(folder_path);

    /// 1. Folder exist?
    if (!fs::exists(input_path) || !fs::is_directory(input_path)) {
        std::cerr << "Input folder '" << folder_path << "' does not exist or is not a directory." << std::endl;
        return gltf_file_paths; /// Return empty vector
    }

    /// 2. Traverse the folder
    for (const auto& entry : fs::directory_iterator(input_path)) {
        /// if not a file, then skip
        if (!entry.is_regular_file()) {
            continue;
        }

        fs::path file_path = entry.path();

        /// 3. Check the extension (.gltf OR .glb)
        /// Lower/Upper case --> make it lower
        std::string ext = file_path.extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(),
            [](unsigned char c) { return std::tolower(c); });

        if (ext == ".gltf" || ext == ".glb") {
            gltf_file_paths.push_back(util::wstring_to_utf8(file_path.wstring()));
        }
    }

    return gltf_file_paths;
}

bool API loadGltf(const std::string& bim_folder, std::vector<chunkbim::MeshChunk>& bimData, const int option)
{
	chunkbim::GltfMesh gltf;
    std::vector<std::string> filePaths = collect_gltf_files(bim_folder);

    size_t numMeshes = 0;
    for (auto& path : filePaths) {
        numMeshes += gltf.getNumMeshes(path);
    }

    bimData.reserve(numMeshes);

    for (auto& path : filePaths) {
        if(!gltf.importMeshData(path, bimData, option))
            return false;
    }

    return true;
}

bool API testGltfExtraId(const std::string& gltfPath)
{
    fs::path input_path(gltfPath);

    if (!fs::exists(input_path)) {
        std::cerr << "Input file '" << gltfPath << "' does not exist.\n";
        return false;
    }

    const fs::path ext = input_path.extension();

    /// ext: .gltf / .glb
    if (ext != ".gltf" && ext != ".glb") {
        return false;        
    }

    chunkbim::GltfMesh::verifyMeshIdsInFile(gltfPath);

    return true;
}