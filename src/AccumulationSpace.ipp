
#include "AccumulationSpace.h"

namespace AccumulationSpace {

// Log level to string conversion
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

// AccumulationLog
AccumulationLog::AccumulationLog(LogLevel level) : logLevel(level), logFileName("AccumuationSpaceLog.txt") {
  logFile.open(logFileName, std::ios::out | std::ios::trunc);
  if (!logFile) {
    std::cerr << "Unable to open log file: " << logFileName << std::endl;
    add(LogLevel::ERROR, "Unable to open log file: ", logFileName, " at ", Timer::now());
  } else {
    add(LogLevel::INFO, "Log file opened successfully.", Timer::now());
  }
}
AccumulationLog::AccumulationLog(const std::string& logFileName, LogLevel level)
    : logFileName(logFileName), logLevel(level) {
  logFile.open(logFileName, std::ios::out | std::ios::trunc);
  if (!logFile) {
    std::cerr << "Unable to open log file: " << logFileName << std::endl;
    add(LogLevel::ERROR, "Unable to open log file: ", logFileName, " at ", Timer::now());
  } else {
    add(LogLevel::INFO, "Log file opened successfully at", Timer::now());
  }
}
AccumulationLog::~AccumulationLog() {
  if (logFile.is_open()) {
    add(LogLevel::INFO, "Log file closed", "at", Timer::now());
    logFile.close();
  }
}
// No need for endl
template <typename... Args>
void AccumulationLog::add(LogLevel level, Args&&... args) {
  if (level >= logLevel) {
    logFile << addLogMessage(logLevelToString(level), ": ", std::forward<Args>(args)...) << std::endl;
  }
}

// Helper function to append log messages
template <typename T>
void AccumulationLog::appendToStream(std::ostringstream& oss, T&& arg) {
  oss << std::forward<T>(arg);
}
template <typename T, typename... Args>
void AccumulationLog::appendToStream(std::ostringstream& oss, T&& first, Args&&... args) {
  oss << std::forward<T>(first);
  appendToStream(oss, std::forward<Args>(args)...);
}
template <typename... Args>
std::string AccumulationLog::addLogMessage(Args&&... args) {
  std::ostringstream oss;
  appendToStream(oss, std::forward<Args>(args)...);
  return oss.str();
}

// AccumulationVoxel
AccumulationVoxel::AccumulationVoxel() {
  position = DGtalPoint3D(0, 0, 0);
  votes = 0;
  confidenceValue = .0f;
  label = 0;
  visited = false;
}

// NormalAccumulationSpace
NormalAccumulationSpace::NormalAccumulationSpace() {
  rangeVoteValue = std::make_pair(0, 0);
  rangeFaceCount = std::make_pair(0, 0);
}
void NormalAccumulationSpace::buildFromFile(const std::string& inputFileName) {
  acclog.add(LogLevel::INFO, "Initiate Nomral Accumulation Space(NAS) from ", inputFileName);

  getVoxelListFromFile(inputFileName);
  getPointsFromVoxelList();
  rangeVoteValue = AccumulationSpace::getMinMaxVotesCountFrom(voxelList);
  rangeFaceCount = AccumulationSpace::getMinMaxFacesCountFrom(voxelList);

  acclog.add(LogLevel::INFO, "Finished building Nomral Accumulation Space(NAS)");
}
void NormalAccumulationSpace::getVoxelListFromFile(const std::string& filename) {

  auto vOrigins = DGtal::PointListReader<Row>::getPolygonsFromFile(filename);
  for (auto l : vOrigins) {
    if (l.size() > 4) {
      voxelList.emplace_back(AccumulationVoxel({l[0][0], l[1][0], l[2][0]}, l[3][0], l[4][0], 0, false));
      AccumulationVoxel& accV = voxelList.back();
      for (unsigned int i = 5; i < l.size(); i++) {
        accV.associatedFaceIds.push_back(l[i][0]);
      }

      std::ostringstream oss;
      oss << " : P:" << accV.position << " | " << "votes:" << accV.votes << " " << "confs:" << accV.confidenceValue
          << " -> ";
      for (auto f : accV.associatedFaceIds) {
        oss << f << " ";
      }

      acclog.add(LogLevel::DEBUG, "<", typeid(*this).name(), ">", oss.str());
    }
  }
}
void NormalAccumulationSpace::getPointsFromVoxelList() {
  for (AccumulationVoxel v : voxelList) {
    pointList.push_back(GLMPoint3D{v.position[0], v.position[1], v.position[2]});
  }
}

// Min/max functions for AccumulationVoxel votes
std::pair<size_t, size_t> getMinMaxVotesCountFrom(const std::vector<AccumulationVoxel>& accList) {
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

std::pair<size_t, size_t> getMinMaxFacesCountFrom(const std::vector<AccumulationVoxel>& accList) {
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

// Hash function for 3D points
size_t accumulationHash(DGtalPoint3D pos) { return std::hash<DGtalPoint3D>{}(pos); }

} // namespace AccumulationSpace