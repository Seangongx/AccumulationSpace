#pragma once

#if defined(AccVoxelHelper_RECURSES)
#error Recursive header files inclusion detected in AccVoxelHelper.h
#else // defined(AccVoxelHelper_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AccVoxelHelper_RECURSES

#if !defined AccVoxelHelper_h
/** Prevents repeated inclusion of headers. */
#define AccVoxelHelper_h

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/PointListReader.h"
#include <array>
#include <limits>

#include "AccVoxel.h"

/// @brief Defined in AccVoxelHelper.h
typedef size_t KeyType;
// #define HelperDebug

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
  static std::vector<AccVoxel> getAccVoxelsFromFile(std::string filename, std::array<KeyType, 2>& AccMinMax) {
    typedef DGtal::PointVector<1, DGtal::int32_t> Point1D;
    std::vector<AccVoxel> resVectAcc;
    std::vector<unsigned int> indexFace;
    auto vOrigins = DGtal::PointListReader<Point1D>::getPolygonsFromFile(filename);
    AccMinMax = {std::numeric_limits<KeyType>::max(), std::numeric_limits<KeyType>::min()};

    for (auto l : vOrigins) {
      if (l.size() > 4) {
        AccVoxel::Point3D pt(l[0][0], l[1][0], l[2][0]);
        AccVoxel accV(pt, l[3][0], l[4][0], 0, false);

        // l.size()-1 to remove the repeated last index in old version of DGtal
        for (unsigned int i = 5; i < l.size(); i++) {
          accV.faces.push_back(l[i][0]);
        }
        resVectAcc.push_back(accV);

        if (accV.votes < AccMinMax[0]) {
          AccMinMax[0] = accV.votes;
        } else {
          if (accV.votes > AccMinMax[1]) {
            AccMinMax[1] = accV.votes;
          }
        }


#ifdef HelperDebug
        std::cout << std::endl;
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

    return resVectAcc;
  }

  /// @brief A function to hash a point
  static KeyType hash(AccVoxel::Point3D p) { return std::hash<AccVoxel::Point3D>{}(p); }
};

#endif // !defined AccVoxelHelper_h

#undef AccVoxelHelper_RECURSES
#endif // else defined(AccVoxelHelper_RECURSES)
