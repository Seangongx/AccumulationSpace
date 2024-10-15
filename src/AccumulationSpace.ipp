#include "AccumulationSpace.h"
#include "DGtal/io/readers/PointListReader.h"
#include "Timer.h"

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
AccumulationLog::AccumulationLog() {
  auto fstream = std::make_shared<std::fstream>(fileName, std::ios::out | std::ios::app);
  init(fileName, level, fstream);
}
AccumulationLog::AccumulationLog(std::shared_ptr<std::fstream> fstream) { init(fileName, LogLevel::INFO, fstream); }
AccumulationLog::AccumulationLog(LogLevel ll, std::shared_ptr<std::fstream> fstream) { init(fileName, ll, fstream); }
AccumulationLog::AccumulationLog(const std::string& logFileName, LogLevel ll, std::shared_ptr<std::fstream> fstream) {
  init(logFileName, ll, fstream);
}
AccumulationLog::~AccumulationLog() {
  if ((*filePtr) && filePtr->is_open()) {
    add(LogLevel::INFO, "Log file closed at ", Timer::now());
    filePtr->close();
  }
}
void AccumulationLog::init(const std::string& logFileName, LogLevel ll, std::shared_ptr<std::fstream> fstream) {
  level = ll;
  filePtr = fstream;
  if (!(*filePtr) || !filePtr->is_open()) {
    std::cerr << "Unable to open log file: " << logFileName << std::endl;
    add(LogLevel::ERROR, "Unable to open log file: ", logFileName, " at ", Timer::now());
  } else {
    add(LogLevel::INFO, "Log file opened successfully.", Timer::now());
  }
}
// No need for endl
template <typename... Args>
void AccumulationLog::add(LogLevel ll, Args&&... args) {
  if (ll >= level) {
    (*filePtr) << addLogMessage(logLevelToString(ll), ": ", std::forward<Args>(args)...) << std::endl;
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
// NormalAccumulationSpace::NormalAccumulationSpace(const NormalAccumulationSpace& other) {
//   voxelList = other.voxelList;
//   pointList = other.pointList;
//   rangeVoteValue = other.rangeVoteValue;
//   rangeFaceCount = other.rangeFaceCount;
//   auto logStream = std::make_shared<std::fstream>(other.log.fileName, std::ios::out | std::ios::app);
//   log.init(other.log.logFile, other.log.fileName, other.log.level);
// }
NormalAccumulationSpace::NormalAccumulationSpace(std::shared_ptr<AccumulationLog> logPtr) : log(logPtr) {
  rangeVoteValue = std::make_pair(0, 0);
  rangeFaceCount = std::make_pair(0, 0);
}
NormalAccumulationSpace::NormalAccumulationSpace(const std::string& inputFileName,
                                                 std::shared_ptr<AccumulationLog> logPtr)
    : log(logPtr) {
  buildFromFile(inputFileName);
}
void NormalAccumulationSpace::buildFromFile(const std::string& inputFileName) {
  log->add(LogLevel::INFO, "Initiate Nomral Accumulation Space(NAS) from ", inputFileName);

  getVoxelListFromFile(inputFileName);
  getPointsFromVoxelList();
  rangeVoteValue = AccumulationSpace::getMinMaxVotesCountFrom(voxelList);
  rangeFaceCount = AccumulationSpace::getMinMaxFacesCountFrom(voxelList);
  log->add(LogLevel::INFO, "Finished building Nomral Accumulation Space(NAS)");
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

      log->add(LogLevel::DEBUG, "<", typeid(*this).name(), log, ">", oss.str());
    }
  }
  log->add(LogLevel::INFO, "Finished reading voxel list from ", filename);
}
void NormalAccumulationSpace::getPointsFromVoxelList() {
  for (AccumulationVoxel v : voxelList) {
    pointList.push_back(GLMPoint3D{v.position[0], v.position[1], v.position[2]});
  }
  log->add(LogLevel::INFO, "Finished converting voxel list to point list");
}

// Min/max functions for AccumulationVoxel votes
std::pair<size_t, size_t> getMinMaxVotesCountFrom(const std::vector<AccumulationVoxel>& accList) {
  if (accList.empty()) {
    throw std::runtime_error("The accumulation list is empty");
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
    throw std::runtime_error("The accumulation list is empty");
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