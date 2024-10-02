#ifndef ACCUMULATIONSPACE_H
#define ACCUMULATIONSPACE_H

/**
 * @file AccumulationSpace.h
 * @brief Define the main data structure and test algorithm for an accumulation space.
 *
 * This file is part of the DGtal library/DGtalTools-contrib Project.
 *
 * @author Bertrand Kerautret (\c bertrand.kerautret@univ-lyon2.fr )
 *         Xun Gong (\c sean.gong814@gmail.com)
 * @date 2024-09-26
 */


///////////////////////////////////////////////////////////////////////////////
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
///////////////////////////////////////////////////////////////////////////////
#include "DGtal/helpers/StdDefs.h"
#include "glm/fwd.hpp"
///////////////////////////////////////////////////////////////////////////////

namespace AccumulationSpace {

// Type Definitions
typedef DGtal::uint32_t DGtalUint;
typedef DGtal::PointVector<1, DGtal::int32_t> Row;
typedef DGtal::Z3i::Point DGtalPoint3D; // Interger 3D point ( Z3i )
typedef glm::vec3 GLMPoint3D;           // Used in polyscope

enum class LogLevel { DEBUG, INFO, WARNING, ERROR };

class AccumulationVoxel;
class AccumulationLog;
class NormalAccumulationSpace;

///////////////////////////////////////////////////////////////////////////////
// Function Declarations
// Function to get the min and max vote counts in resVectAcc
static std::pair<size_t, size_t> getMinMaxVotesCountFrom(const std::vector<AccumulationVoxel>& accList);
// Function to get the min and max face counts in resVectAcc
static std::pair<size_t, size_t> getMinMaxFacesCountFrom(const std::vector<AccumulationVoxel>& accList);

std::string logLevelToString(LogLevel level) {
  switch (level) {
  case LogLevel::DEBUG:
    return "DEBUG";
  case LogLevel::INFO:
    return "INFO";
  case LogLevel::WARNING:
    return "WARNING";
  case LogLevel::ERROR:
    return "ERROR";
  default:
    return "UNKNOWN";
  }
}

class AccumulationLog {
public:
  AccumulationLog(LogLevel level = LogLevel::INFO) : logLevel(level) {
    logFile.open(logFileName, std::ios::out | std::ios::trunc);
    if (!logFile) {
      std::cerr << "Unable to open " << logFileName << std::endl;
      log(LogLevel::ERROR, "Unable to open ", logFileName);
    } else {
      log(LogLevel::INFO, logFileName, " opened successfully.");
    }
  }

  AccumulationLog(const std::string& logFileName, LogLevel level = LogLevel::INFO)
      : logFileName(logFileName), logLevel(level) {
    logFile.open(logFileName, std::ios::out | std::ios::trunc);
    if (!logFile) {
      std::cerr << "Unable to open " << logFileName << std::endl;
      log(LogLevel::ERROR, "Unable to open log file: ", logFileName);
    } else {
      log(LogLevel::INFO, logFileName, " opened successfully.");
    }
  }

  ~AccumulationLog() {
    if (logFile.is_open()) {
      log(LogLevel::INFO, logFileName, " closed\n");
      logFile.close();
    }
  }

  template <typename... Args>
  // No need for endl
  void log(LogLevel level, Args&&... args) {
    if (level >= logLevel) {
      logFile << addLogMessage(logLevelToString(level), ": ", std::forward<Args>(args)...) << std::endl;
    }
  }

private:
  std::ofstream logFile;                              // log file stream
  std::string logFileName{"AccumuationSpaceLog.txt"}; // log file name
  LogLevel logLevel;                                  // log current level

  // // C++17 fold expression
  // template <typename... Args>
  // std::string addLogMessage(Args&&... args) {
  //   std::ostringstream oss;
  //   (oss << ... << args); // C++17 fold expression
  //   return oss.str();
  // }
  // Recursive template function
  template <typename T>
  void appendToStream(std::ostringstream& oss, T&& arg) {
    oss << std::forward<T>(arg);
  }
  template <typename T, typename... Args>
  void appendToStream(std::ostringstream& oss, T&& first, Args&&... args) {
    oss << std::forward<T>(first);
    appendToStream(oss, std::forward<Args>(args)...);
  }
  template <typename... Args>
  std::string addLogMessage(Args&&... args) {
    std::ostringstream oss;
    appendToStream(oss, std::forward<Args>(args)...);
    return oss.str();
  }
};

// @brief The class to store unit information for each voxel
class AccumulationVoxel {
public:
  DGtalPoint3D position;
  DGtalUint votes = 0;
  DGtalUint label = 0;
  float confidenceValue = .0f;
  bool visited = false;
  std::vector<DGtalUint> associatedFaceIds;

  AccumulationVoxel() {
    position = DGtalPoint3D(0, 0, 0);
    votes = 0;
    confidenceValue = .0f;
    label = 0;
    visited = false;
  }
  AccumulationVoxel(DGtalPoint3D _p, DGtalUint _v, DGtalUint _c, DGtalUint _l, bool _f)
      : position(_p), votes(_v), confidenceValue(_c), label(_l), visited(_f) {}

  /// @brief Defualt compare operator for accumulation votes
  /// @param rhs
  /// @return
  bool operator<(const AccumulationVoxel& rhs) const { return votes < rhs.votes; }
};


/**
 * @class NormalAccumulationSpace
 * @brief The normal accumulation space of a shape
 */
class NormalAccumulationSpace {
public:
  NormalAccumulationSpace() {}
  NormalAccumulationSpace(const std::string& inputFileName) { buildFromFile(inputFileName); };
  ~NormalAccumulationSpace(){};

  void buildFromFile(const std::string& inputFileName) {
    acclog.log(LogLevel::INFO, "Initiate Nomral Accumulation Space(NAS) from ", inputFileName);

    getVoxelListFromFile(inputFileName);
    getPointsFromVoxelList();
    rangeVoteValue = AccumulationSpace::getMinMaxVotesCountFrom(accVoxelList);
    rangeFaceCount = AccumulationSpace::getMinMaxFacesCountFrom(accVoxelList);

    acclog.log(LogLevel::INFO, "Finished building NAS");
  }

  std::vector<AccumulationVoxel> accVoxelList;
  std::vector<GLMPoint3D> accPointList;
  std::pair<size_t, size_t> rangeVoteValue;
  std::pair<size_t, size_t> rangeFaceCount;

private:
  AccumulationLog acclog;

  /// @brief Read **extracted file from CDCVAM** includes accumulation and confidence \n
  /// @brief Warning: using old version of DGtal may read 1 more repeated last face index
  /// @param filename
  /// @return
  void getVoxelListFromFile(const std::string& filename) {

    auto vOrigins = DGtal::PointListReader<Row>::getPolygonsFromFile(filename);
    for (auto l : vOrigins) {
      if (l.size() > 4) {
        accVoxelList.emplace_back(AccumulationVoxel({l[0][0], l[1][0], l[2][0]}, l[3][0], l[4][0], 0, false));
        AccumulationVoxel& accV = accVoxelList.back();
        for (unsigned int i = 5; i < l.size(); i++) {
          accV.associatedFaceIds.push_back(l[i][0]);
        }

        std::ostringstream oss;
        oss << " : P:" << accV.position << " | " << "votes:" << accV.votes << " " << "confs:" << accV.confidenceValue
            << " -> ";
        for (auto f : accV.associatedFaceIds) {
          oss << f << " ";
        }

        acclog.log(LogLevel::INFO, typeid(*this).name(), oss.str());
      }
    }
  }

  void getPointsFromVoxelList() {
    for (AccumulationVoxel v : accVoxelList) {
      accPointList.push_back(GLMPoint3D{v.position[0], v.position[1], v.position[2]});
    }
  }
};


///////////////////////////////////////////////////////////////////////////////
// Helper functions
static std::pair<size_t, size_t> getMinMaxVotesCountFrom(const std::vector<AccumulationVoxel>& accList) {
  if (accList.empty()) {
    throw std::runtime_error("The vector is empty");
  }

  auto minElem =
      std::min_element(accList.begin(), accList.end(),
                       [&](const AccumulationVoxel& a, const AccumulationVoxel& b) { return a.votes < b.votes; });
  auto maxElem =
      std::max_element(accList.begin(), accList.end(),
                       [&](const AccumulationVoxel& a, const AccumulationVoxel& b) { return a.votes < b.votes; });

  return {minElem->votes, maxElem->votes};
}

static std::pair<size_t, size_t> getMinMaxFacesCountFrom(const std::vector<AccumulationVoxel>& accList) {
  if (accList.empty()) {
    throw std::runtime_error("The vector is empty");
  }

  auto faceSize = [](const AccumulationVoxel& accV) { return accV.associatedFaceIds.size(); };
  auto minElem =
      std::min_element(accList.begin(), accList.end(), [&](const AccumulationVoxel& a, const AccumulationVoxel& b) {
        return faceSize(a) < faceSize(b);
      });
  auto maxElem =
      std::max_element(accList.begin(), accList.end(), [&](const AccumulationVoxel& a, const AccumulationVoxel& b) {
        return faceSize(a) < faceSize(b);
      });

  return {minElem->associatedFaceIds.size(), maxElem->associatedFaceIds.size()};
}

/// @brief A function to hash a accumulation position
static size_t accumulationHash(AccumulationSpace::DGtalPoint3D pos) {
  return std::hash<AccumulationSpace::DGtalPoint3D>{}(pos);
}

} // namespace AccumulationSpace


#endif // ACCUMULATIONSPACE_H