#include "pc3d.h"

namespace pctree
{
	bool buildKdtree3D(const pctree::PointCloudType3D& pc, std::shared_ptr<pctree::kd_tree3D>& tree)
	{
		/// Build tree
		try {
			tree.reset(new pctree::kd_tree3D(pctree::PointCloudType3D::dim, pc, nanoflann::KDTreeSingleIndexAdaptorParams(maxLeaf)));
			tree->buildIndex();
		}
		catch (...) {
			std::cerr << "\nError in buildTree of KDTree" << std::endl;
			return false;
		}

		return true;
	}

	bool saveTree3D(const std::shared_ptr<pctree::kd_tree3D>& tree, const std::string& treePath)
	{
		FILE* idxFile = nullptr;
		if (0 != fopen_s(&idxFile, treePath.c_str(), "wb"))
		{
			std::cerr << "Error in creating a index file, " << treePath << "\n";
			return false;
		}

		tree->saveIndex(idxFile);

		fclose(idxFile);

		return true;
	}

	bool buildAndSaveKdtree3D(const pctree::PointCloudType3D& pc, const std::string& treePath)
	{
		/// Build tree
		std::shared_ptr<pctree::kd_tree3D> tree;
		if (!buildKdtree3D(pc, tree))
			return false;

		FILE* idxFile = nullptr;
		if (0 != fopen_s(&idxFile, treePath.c_str(), "wb"))
			throw std::runtime_error(std::string("Error in creating a index file, ") + treePath);

		tree->saveIndex(idxFile);

		fclose(idxFile);

		return true;
	}

	bool loadKdtree3D(std::shared_ptr<pctree::kd_tree3D>& tree, const pctree::PointCloudType3D& pc, const std::string& treePath)
	{
		try {
			tree.reset();
			tree = std::make_shared<pctree::kd_tree3D>(pctree::PointCloudType3D::dim, pc, nanoflann::KDTreeSingleIndexAdaptorParams(maxLeaf));
			FILE* idxFile = nullptr;
			if (0 != fopen_s(&idxFile, treePath.c_str(), "rb"))
				throw std::runtime_error(std::string("Error in opening a index file, ") + treePath);
			tree->loadIndex(idxFile);
			fclose(idxFile);
		}
		catch (...) {
			std::cerr << "Error in loading a kdtree index file, " << treePath << "\n";
			return false;
		}

		return true;
	}
}