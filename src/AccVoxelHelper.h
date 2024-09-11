#pragma once

#include <cstddef>
#if defined(AccVoxelHelper_RECURSES)
#error Recursive header files inclusion detected in AccVoxelHelper.h
#else // defined(AccVoxelHelper_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AccVoxelHelper_RECURSES

#if !defined AccVoxelHelper_h
/** Prevents repeated inclusion of headers. */
#define AccVoxelHelper_h

#include "AccVoxel.h"
#include "DGtal/io/readers/PointListReader.h"

/// @brief Defined in AccVoxelHelper.h
typedef size_t KeyType;
typedef DGtal::PointVector<1, DGtal::int32_t> Point1D;
// #define HelperDebugInfo

struct CompareAccAsc {
  bool operator()(const AccVoxel& e1, const AccVoxel& e2) const { return e1.votes < e2.votes; }
};

struct CompareConfsAsc {
  bool operator()(const AccVoxel& e1, const AccVoxel& e2) const { return e1.confs < e2.confs; }
};

class AccVoxelHelper {
public:
  /// @brief Read extracted file includes accumulation and confidence \n
  /// @brief Warning: using old version of DGtal may read 1 more repeated last face index
  /// @param filename
  /// @return
  static std::vector<AccVoxel> getAccVoxelsFromFile(const std::string& filename) {
    std::vector<AccVoxel> resVectAcc;
    std::vector<DGtal::uint32_t> indexFace;
    auto vOrigins = DGtal::PointListReader<Point1D>::getPolygonsFromFile(filename);

    for (auto l : vOrigins) {
      if (l.size() > 4) {
        resVectAcc.emplace_back(AccVoxel({l[0][0], l[1][0], l[2][0]}, l[3][0], l[4][0], 0, false));
        AccVoxel& accV = resVectAcc.back();
        //  l.size()-1 to remove the repeated last index in old version of DGtal
        for (unsigned int i = 5; i < l.size(); i++) {
          accV.faces.push_back(l[i][0]);
        }
#ifdef HelperDebugInfo
        std::cout << "<HelperDebugInfo>" std::endl;
        std::cout << "P:" << accV.p << " | ";
        std::cout << "votes:" << accV.votes << " ";
        std::cout << "confs:" << accV.confs << " -> ";
        for (auto f : accV.faces) {
          std::cout << f << " ";
        }
        std::cout << std::endl;
#endif
      }
    }

    // RVO，implicit std::move
    return resVectAcc;
  }


  // Function to get the min and max face counts in resVectAcc
  static std::pair<size_t, size_t> getMinMaxFacesCount(const std::vector<AccVoxel>& accList) {
    if (accList.empty()) {
      throw std::runtime_error("The vector is empty");
    }

    // Lambda function to get the size of faces
    auto faceSize = [](const AccVoxel& accV) { return accV.faces.size(); };

    // Get min and max AccVoxel based on the size of the faces vector
    auto minElem = std::min_element(accList.begin(), accList.end(),
                                    [&](const AccVoxel& a, const AccVoxel& b) { return faceSize(a) < faceSize(b); });

    auto maxElem = std::max_element(accList.begin(), accList.end(),
                                    [&](const AccVoxel& a, const AccVoxel& b) { return faceSize(a) < faceSize(b); });

    // Return the sizes (min, max)
    return {minElem->faces.size(), maxElem->faces.size()};
  }

  /// @brief A function to hash a point
  static KeyType accumulationHash(AccVoxel::Point3D p) { return std::hash<AccVoxel::Point3D>{}(p); }
};

#endif // !defined AccVoxelHelper_h

#undef AccVoxelHelper_RECURSES
#endif // else defined(AccVoxelHelper_RECURSES)
