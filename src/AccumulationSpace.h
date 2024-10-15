#ifndef ACCUMULATIONSPACE_H
#define ACCUMULATIONSPACE_H

/**
 * @file AccumulationSpace.h
 * @brief Define the main data structure and test algorithm for an accumulation
 * space.
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
#include <memory>
#include <sstream>
#include <string>
#include <vector>
///////////////////////////////////////////////////////////////////////////////
#include "DGtal/helpers/StdDefs.h"
#include "glm/fwd.hpp"
#include "glm/glm.hpp"
#include "glm/vec3.hpp"
///////////////////////////////////////////////////////////////////////////////

namespace AccumulationSpace {

// Type Definitions
typedef DGtal::uint32_t DGtalUint;
typedef DGtal::PointVector<1, DGtal::int32_t> Row;
typedef DGtal::Z3i::Point DGtalPoint3D; // Interger 3D point ( Z3i )
typedef glm::vec3 GLMPoint3D;           // Used in polyscope

enum class LogLevel { DEBUG, INFO, WARNING, ERROR };

class AccumulationVoxel;       // Forward declaration
class AccumulationLog;         // Forward declaration
class NormalAccumulationSpace; // Forward declaration

// Function Declarations
std::string logLevelToString(LogLevel level);
std::pair<size_t, size_t> getMinMaxVotesCountFrom(const std::vector<AccumulationVoxel>& accList);
std::pair<size_t, size_t> getMinMaxFacesCountFrom(const std::vector<AccumulationVoxel>& accList);
size_t accumulationHash(DGtalPoint3D pos);

///////////////////////////////////////////////////////////////////////////////
/**
 * @class AccumulationLog
 * @brief The class to print and save log messages
 */
class AccumulationLog {
public:
  AccumulationLog();
  AccumulationLog(std::shared_ptr<std::fstream> fstream);
  AccumulationLog(std::shared_ptr<std::fstream> fstream, LogLevel ll);
  AccumulationLog(std::shared_ptr<std::fstream> fstream, LogLevel ll, const std::string& logFileName);
  ~AccumulationLog();
  void init(std::shared_ptr<std::fstream> fstream, const std::string& logFileName, LogLevel ll);

  template <typename... Args>
  void add(LogLevel level, Args&&... args);

  std::shared_ptr<std::fstream> logFile;
  std::string fileName = "AccumulationSpaceLog.txt";
  LogLevel level = LogLevel::INFO;

private:
  template <typename T>
  void appendToStream(std::ostringstream& oss, T&& arg);
  template <typename T, typename... Args>
  void appendToStream(std::ostringstream& oss, T&& first, Args&&... args);
  template <typename... Args>
  std::string addLogMessage(Args&&... args);
};

///////////////////////////////////////////////////////////////////////////////
/**
 * @class AccumulationVoxel
 * @brief The class to store unit information for each voxel
 */
class AccumulationVoxel {
public:
  DGtalPoint3D position = DGtalPoint3D(0, 0, 0);
  DGtalUint votes = 0;
  DGtalUint label = 0;
  float confidenceValue = .0f;
  bool visited = false;
  std::vector<DGtalUint> associatedFaceIds;

  AccumulationVoxel();
  AccumulationVoxel(DGtalPoint3D _p, DGtalUint _v, DGtalUint _c, DGtalUint _l, bool _f)
      : position(_p), votes(_v), confidenceValue(_c), label(_l), visited(_f) {}

  /// @brief Defualt compare operator for accumulation votes
  /// @param rhs
  /// @return
  bool operator<(const AccumulationVoxel& rhs) const { return votes < rhs.votes; }
};

///////////////////////////////////////////////////////////////////////////////
/**
 * @class NormalAccumulationSpace
 * @brief The normal accumulation space of a shape
 */
class NormalAccumulationSpace {
public:
  NormalAccumulationSpace();
  // NormalAccumulationSpace(const NormalAccumulationSpace& other);
  NormalAccumulationSpace(std::shared_ptr<AccumulationLog> logPtr);
  NormalAccumulationSpace(const std::string& inputFileName, std::shared_ptr<AccumulationLog> logPtr);
  ~NormalAccumulationSpace(){};

  void buildFromFile(const std::string& inputFileName);

  std::vector<AccumulationVoxel> voxelList;
  std::vector<GLMPoint3D> pointList;
  std::pair<size_t, size_t> rangeVoteValue;
  std::pair<size_t, size_t> rangeFaceCount;
  std::shared_ptr<AccumulationLog> log;

private:
  /// @brief Read **extracted file from CDCVAM** includes accumulation and
  /// confidence \n
  /// @brief Warning: using old version of DGtal may read 1 more repeated last
  /// face index
  /// @param filename
  /// @return
  void getVoxelListFromFile(const std::string& filename);
  void getPointsFromVoxelList();
};

} // namespace AccumulationSpace

#endif // ACCUMULATIONSPACE_H