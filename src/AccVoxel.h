#pragma once

#if defined(AccVoxel_RECURSES)
#error Recursive header files inclusion detected in AccVoxel.h
#else // defined(AccVoxel_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AccVoxel_RECURSES

#if !defined AccVoxel_h
/** Prevents repeated inclusion of headers. */
#define AccVoxel_h

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include <fstream>
#include <unordered_map>


typedef DGtal::uint32_t Uint;

// @brief A structure to store a voxel in max-heap
class AccVoxel {
public:
  typedef DGtal::uint32_t Uint;
  typedef DGtal::Z3i::Point Point3D; // Interger 3D point ( Z3i )

public:
  Point3D p;
  Uint votes;
  float confs;
  Uint label;
  bool visited;
  std::vector<Uint> faces;

  AccVoxel() {
    p = Point3D(0, 0, 0);
    votes = 0;
    confs = 0;
    label = 0;
    visited = false;
  }
  AccVoxel(Point3D _p, Uint _v, Uint _c, Uint _l, bool _f) : p(_p), votes(_v), confs(_c), label(_l), visited(_f) {}

  /// @brief Defualt compare operator for accumulation votes
  /// @param rhs
  /// @return
  bool operator<(const AccVoxel& rhs) const { return votes < rhs.votes; }
};

class AccLog {

  std::ofstream logFile;                         // log file stream
  std::string logFileName{"AccumuationLog.txt"}; // log file name

public:
  AccLog() {
    logFile.open(logFileName, std::ios::out | std::ios::trunc); // Using trunc to overwrite
    if (!logFile) {
      std::cerr << "Unable to open log file: " << logFileName << std::endl;
    } else {
      logFile << "Log file opened successfully." << std::endl;
    }
  }
  ~AccLog() {
    if (logFile.is_open()) {
      logFile.close();
    }
  }
  void addEnrty(const std::string& entry) { logFile << entry << std::endl; }
};

#endif // !defined AccVoxel_h

#undef AccVoxel_RECURSES
#endif // else defined(AccVoxel_RECURSES)
