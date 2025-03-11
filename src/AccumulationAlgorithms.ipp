#include <cstddef>
#include <queue>
#include <system_error>
#include <unordered_map>
#include <vector>
#include "AccumulationAlgorithms.h"
#include "Timer.h"

namespace AccumulationAlgorithms {

void ClusterAlgoBase::clearCluster() {
  voxelMap.clear();
  cluster.clear();
  clusterLabel = 0;
  accPQ = std::priority_queue<AccVoxel>();
  if (log != nullptr) {
    log->add(LogLevel::INFO, "CLEAN: ", typeid(*this).name());
  }
}
ClusterAlgoBase::~ClusterAlgoBase() {
  clearCluster();
}
// Check if the key exists in a template map
template <typename MapType, typename KeyType>
bool ClusterAlgoBase::mapKeychecker(const MapType& map, const KeyType& key) {
  auto it = map.find(key);
  if (it != map.end())
    return true;
  return false;
}
// Generate neighborhoods based on the ring of neighbors
std::vector<std::tuple<int, int, int>> ClusterAlgoBase::generateVoxelNeighbors(
    int ring) {
  std::vector<std::tuple<int, int, int>> neighbors;
  for (int dx = -ring; dx <= ring; ++dx) {
    for (int dy = -ring; dy <= ring; ++dy) {
      for (int dz = -ring; dz <= ring; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0)
          continue;
        // Manhattan distance
        // int distance = std::abs(dx) + std::abs(dy) + std::abs(dz);
        // if (distance <= ring) {
        //   neighbors.emplace_back(dx, dy, dz);
        // }
        // Euclidean distance
        int distanceSquared = dx * dx + dy * dy + dz * dz;
        if (distanceSquared <= ring * ring) {
          neighbors.emplace_back(dx, dy, dz);
        }
      }
    }
  }
  return neighbors;
}

void ClusterAlgoBase::markVoxelNeighbours(const AccVoxel& voxel,
                                          std::queue<AccVoxel>& queue,
                                          int ring) {
  std::vector<std::tuple<int, int, int>> neighbors =
      generateVoxelNeighbors(ring);

  for (const auto& [dx, dy, dz] : neighbors) {
    DGtalPoint3D posTemp(voxel.position[0] + dx, voxel.position[1] + dy,
                         voxel.position[2] + dz);
    auto idTemp = accumulationHash(posTemp);
    // check if the key exists in the map
    if (mapKeychecker(voxelMap, idTemp)) {
      auto& voxelTemp = voxelMap.at(idTemp);
      // skip visied/centre/flag!=0 voxel
      if (!voxelTemp.visited && posTemp != voxel.position &&
          voxelTemp.label == 0) {
        voxelTemp.label = voxel.label;
        queue.push(voxelTemp);
        log->add(LogLevel::DEBUG, "Label in [", voxel.label, "] and push ",
                 voxelTemp.position);
      }
    }
  }
}

void NeighbourClusterAlgo::clearCluster() {
  ClusterAlgoBase::clearCluster();
}
NeighbourClusterAlgo::~NeighbourClusterAlgo() {
  clearCluster();
}
NeighbourClusterAlgo::NeighbourClusterAlgo(
    NormalAccumulationSpace& normalAccSpace, size_t thAcc, double thConf,
    std::shared_ptr<AccumulationLog> logPtr) {
  voxelList = normalAccSpace.voxelList;
  // Set initial ring to 1
  buildCluster(voxelList, thAcc, thConf, logPtr, 1);
}
void NeighbourClusterAlgo::buildCluster(std::vector<AccVoxel>& accList,
                                        size_t thAcc, double thConf,
                                        std::shared_ptr<AccumulationLog> logPtr,
                                        int ring) {
  log = logPtr;
  for (auto& v : accList) {
    auto hashValue = AccumulationSpace::accumulationHash(v.position);
    voxelMap[hashValue] = v;
  }
  log->add(LogLevel::INFO, "START: ", typeid(*this).name(),
           " with accumulation thereshold: ", accThreshold,
           " , confidence threshold: ", confThreshold, " and ring: ", ring);
  Timer timer(typeid(*this).name());
  timer.start();
  markNRingAccLabel(ring);
  timer.stop();
  log->add(LogLevel::INFO, timer.log());
}

void NeighbourClusterAlgo::markNRingAccLabel(int ring) {
  std::queue<AccVoxel> localQ;                  // maintain the local visited:
  cluster.push_back(std::vector<DGtalUint>());  // init the empty cluster
                                                // 0(cluster 0 doesn't be used)
  DGtalUint countPush = 0;

  for (auto v : voxelMap)
    accPQ.push(v.second);
  log->add(LogLevel::INFO, "Loaded global queue size is ", accPQ.size());

  // LOOP I: select the next voxel with the highest votes
  while (!accPQ.empty()) {
    auto vCurrent = accPQ.top();
    auto pCurrent = vCurrent.position;
    auto idCurrent = accumulationHash(vCurrent.position);

    if (voxelMap.at(idCurrent).visited) {
      accPQ.pop();
      log->add(LogLevel::DEBUG, "Pop(G): ", pCurrent, " and remain ",
               accPQ.size());
      continue;
    }

    // LOOP II: traverse the adjacent voxels

    // Form a new cluster, push the first voxel and update the current cluster
    // label
    voxelMap.at(idCurrent).visited = true;
    voxelMap.at(idCurrent).label = ++clusterLabel;
    vCurrent.label = clusterLabel;
    cluster.push_back(std::vector<DGtalUint>());
    cluster[clusterLabel].push_back(idCurrent);

    log->add(LogLevel::DEBUG, "(", ++countPush, ") push ",
             voxelMap.at(idCurrent).position, " in cluster ", clusterLabel,
             " and set visited");
    log->add(LogLevel::DEBUG, "Current cluster center is ", pCurrent, " -> ",
             vCurrent.label);
    log->add(LogLevel::DEBUG, "Remain(G): ", accPQ.size());

    // markNeighbours(vCurrent, mapVoxel, localQ);
    markVoxelNeighbours(vCurrent, localQ, ring);
    while (!localQ.empty()) {
      auto vAdjacent = localQ.front();
      auto pAdjacent = vAdjacent.position;
      auto idAdjacent = accumulationHash(vAdjacent.position);
      log->add(LogLevel::DEBUG, "Visiting [", clusterLabel, "] : ", pAdjacent);
      log->add(LogLevel::DEBUG, "Remain(L): ", localQ.size());

      // mark visited, store in a cluster and label rest neighbors
      voxelMap.at(idAdjacent).visited = true;
      cluster[clusterLabel].push_back(idAdjacent);
      localQ.pop();
      markVoxelNeighbours(vAdjacent, localQ, ring);
    }
  }
  log->add(LogLevel::INFO, "FINISH: Found ", clusterLabel, " clusters");
}

void RadiusClusterAlgo::clearCluster() {
  ClusterAlgoBase::clearCluster();
}
RadiusClusterAlgo::~RadiusClusterAlgo() {
  clearCluster();
}
void RadiusClusterAlgo::buildCluster(std::vector<AccVoxel>& accList,
                                     size_t thAcc, double thConf,
                                     std::shared_ptr<AccumulationLog> logPtr,
                                     const PolySurface& polysurface, char mode,
                                     int radius) {
  log = logPtr;
  voxelList = accList;
  surf = polysurface;
  radiusDepth = radius;

  log->add(LogLevel::INFO, "START: ", typeid(*this).name(),
           " with accumulation thereshold: ", accThreshold,
           " and confidence threshold: ", confThreshold);
  for (auto& v : accList) {
    auto hashValue = accumulationHash(v.position);
    voxelMap[hashValue] = v;
  }
  log->add(LogLevel::INFO, "Starting building voxelMap with ", voxelList.size(),
           " voxels");
  for (size_t i = 0; i < voxelList.size(); i++) {
    const auto& voxel = voxelList[i];
    for (auto faceId : voxel.associatedFaceIds) {
      faceMap[faceId].push_back(i);
    }
  }

  Timer timer("RadiusCluster");
  timer.start();

  switch (mode) {
    case 0:
      markStaticRadiusAccLabel();
      break;
    case 1:
      markDynamicRadiusAccLabel();
      break;
    case 2:
      markAdaptiveRadiusAccLabel();
      break;
    default:
      log->add(LogLevel::ERROR, "Invalid mode");
  }

  timer.stop();
  log->add(LogLevel::INFO, timer.log());
}

void RadiusClusterAlgo::markRadiusNeighbours(const AccVoxel& voxel,
                                             std::queue<AccVoxel>& queue) {
  float avgRadius = getAccumulationAverageRadius(voxel);
  std::vector<std::tuple<int, int, int>> neighbors = generateVoxelNeighbors(1);

  for (const auto& [dx, dy, dz] : neighbors) {
    DGtalPoint3D posTemp(voxel.position[0] + dx, voxel.position[1] + dy,
                         voxel.position[2] + dz);
    auto idTemp = AccumulationSpace::accumulationHash(posTemp);

    if (mapKeychecker(voxelMap, idTemp)) {
      auto& voxelTemp = voxelMap.at(idTemp);
      // skip visied/centre/flag!=0 voxel
      if (!voxelTemp.visited && posTemp != voxel.position &&
          voxelTemp.label == 0) {
        float dis = glm::distance(
            glm::vec3(voxelTemp.position[0], voxelTemp.position[1],
                      voxelTemp.position[2]),
            glm::vec3(voxel.position[0], voxel.position[1], voxel.position[2]));
        voxelTemp.label = voxel.label;
        queue.push(voxelTemp);
        log->add(LogLevel::DEBUG, "Label in [", voxel.label, "] and push ",
                 voxelTemp.position);
      }
    }
  }
}

// In DGtal space
DGtal::Z3i::RealPoint RadiusClusterAlgo::getFaceBarycenter(
    const PolySurface::Face& aFace) {
  DGtal::Z3i::RealPoint res(0.0, 0.0, 0.0);
  for (auto const& v : surf.verticesAroundFace(aFace)) {
    res += surf.position(v);
  }
  return res / surf.verticesAroundFace(aFace).size();
}
float RadiusClusterAlgo::getAccumulationAverageRadius(const AccVoxel& voxel) {
  auto voxelPos = voxel.position;
  float radius = 0.1f;
  for (auto fid : voxel.associatedFaceIds) {
    // 1. get the face center
    auto faceBarycenter = getFaceBarycenter(fid);
    // 2. get the distance between the face center and the current voxel
    auto distance = glm::distance(
        glm::vec3(faceBarycenter[0], faceBarycenter[1], faceBarycenter[2]),
        glm::vec3(voxelPos[0], voxelPos[1], voxelPos[2]));
    radius += distance;
  }
  radius = radius / voxel.associatedFaceIds.size();
  log->add(LogLevel::DEBUG, "DEBUG: Get the average radius of the voxel ",
           voxelPos[0], " ", voxelPos[1], " ", voxelPos[2], " which has ",
           voxel.associatedFaceIds.size(), " faces and distance is ", radius);
  return radius;
}

void RadiusClusterAlgo::markStaticRadiusAccLabel() {

  std::queue<AccVoxel> localQ;                  // maintain the local visited:
  cluster.push_back(std::vector<DGtalUint>());  // init the empty cluster
                                                // 0(cluster 0 doesn't be used)
  DGtalUint countPush = 0;

  for (auto v : voxelMap)
    accPQ.push(v.second);
  log->add(LogLevel::INFO, "START: SRA load global queue size is ",
           accPQ.size());

  // LOOP I: get a voxel with the current highest votes from the PQ
  while (!accPQ.empty()) {
    auto vCurrent = accPQ.top();
    auto pCurrent = vCurrent.position;
    auto idCurrent = accumulationHash(vCurrent.position);

    if (voxelMap.at(idCurrent).visited) {
      accPQ.pop();
      log->add(LogLevel::DEBUG, "Pop(G): ", pCurrent, " and remain ",
               accPQ.size());
      continue;
    }

    // Form a new cluster, push the first voxel and update the current cluster
    // label
    voxelMap.at(idCurrent).visited = true;
    voxelMap.at(idCurrent).label = ++clusterLabel;
    vCurrent.label = clusterLabel;
    cluster.push_back(std::vector<DGtalUint>());
    cluster[clusterLabel].push_back(idCurrent);

    log->add(LogLevel::DEBUG, "(", ++countPush, ") push ",
             voxelMap.at(idCurrent).position, " in cluster ", clusterLabel,
             " and set visited");
    log->add(LogLevel::DEBUG, "Current cluster center is ", pCurrent, " -> ",
             vCurrent.label);
    log->add(LogLevel::DEBUG, "Remain(G): ", accPQ.size());

    // LOOP II: traverse the adjacent voxels

    int radius =
        static_cast<int>(std::round(getAccumulationAverageRadius(vCurrent)));

    markVoxelNeighbours(vCurrent, localQ, radius);  // static radius
    // markRadiusNeighbours(vCurrent, mapVoxel, localQ, surf); // dynamic radius

    while (!localQ.empty()) {
      auto vAdjacent = localQ.front();
      auto pAdjacent = vAdjacent.position;
      auto idAdjacent = AccumulationSpace::accumulationHash(vAdjacent.position);

      log->add(LogLevel::DEBUG, "Visiting [", clusterLabel, "] : ", pAdjacent);
      log->add(LogLevel::DEBUG, "Remain(L): ", localQ.size());

      if (voxelMap.at(idAdjacent).visited) {
        localQ.pop();
        continue;
      }
      if (voxelMap.at(idAdjacent).label == 0) {
        log->add(LogLevel::ERROR, pAdjacent, " is not visited and no label");
      }

      // mark visited, store in a cluster and label rest neighbors
      voxelMap.at(idAdjacent).visited = true;
      cluster[clusterLabel].push_back(idAdjacent);
      localQ.pop();
    }
  }
  log->add(LogLevel::INFO, "FINISH: Found ", clusterLabel, " clusters");
}

// In DGtal space
// Check if the face f is sharing edge with the current faceId
bool RadiusClusterAlgo::isSharingEdge(PolySurface::Face faceId,
                                      PolySurface::Face f) {
  auto v1 = surf.verticesAroundFace(faceId);
  auto v2 = surf.verticesAroundFace(f);
  int count = 0;
  for (auto const& v : v1) {
    if (std::find(v2.begin(), v2.end(), v) != v2.end()) {
      count++;
    }
  }
  return count == 2;
}
std::vector<PolySurface::Face> RadiusClusterAlgo::getNeighborFacesByFaceId(
    PolySurface::Face faceId, size_t ring) {

  std::vector<PolySurface::Face> result;
  std::unordered_set<PolySurface::Face> fVisited;

  // 获取当前面的三个顶点
  for (const auto& v : surf.verticesAroundFace(faceId)) {
    log->add(LogLevel::DEBUG, "处理面 ", faceId, " 的一个顶点 ", v);
    for (const auto& f : surf.facesAroundVertex(v)) {
      if (f != faceId && isSharingEdge(faceId, f)) {
        if (!fVisited.count(f)) {
          fVisited.insert(f);
          result.push_back(f);
          log->add(LogLevel::DEBUG, "插入面 ", f, " 作为 ", faceId, " 的邻居");
        }
      }
    }
  }
  if (result.size() == 0) {
    log->add(LogLevel::ERROR, faceId, " 面居然没有找到邻居");
  } else if (result.size() > 0) {
    log->add(LogLevel::DEBUG, "Found ", result.size(),
             " neighbor faces for face ", faceId);
  }
  return result;
}

void RadiusClusterAlgo::pushValidCentervoxelAssociatedVoxelsIntoQueue(
    const AccVoxel& centerVoxel, std::queue<AccVoxel>& queue, int ring) {

  for (auto f : centerVoxel.associatedFaceIds) {
    auto neighborFaces = getNeighborFacesByFaceId(f, ring);
    for (auto nf : neighborFaces) {
      auto itf = faceMap.find(nf);
      if (itf == faceMap.end()) {
        log->add(LogLevel::ERROR, nf, " 没找到的对应faceMap");
        continue;
      }
      for (auto vid : itf->second) {
        log->add(LogLevel::DEBUG, "面 ", nf, " 对应voxel hash ", vid);
        size_t ahv = accumulationHash(voxelList[vid].position);
        auto itv = voxelMap.find(ahv);
        if (itv == voxelMap.end()) {
          log->add(LogLevel::ERROR, "居然有根据面 ", nf,
                   " 没找到的对应voxel的情况");
          continue;
        }
        // skip visied/centre/flag!=0 voxel
        auto& voxelTemp = itv->second;
        auto& posTemp = voxelTemp.position;
        if (!voxelTemp.visited && posTemp != centerVoxel.position &&
            voxelTemp.label == 0) {
          // 可能会有和之前queue内重复的情况，但是我们的外部算法有判别
          voxelTemp.label = centerVoxel.label;
          queue.push(itv->second);
        }
      }
    }
  }
}

void RadiusClusterAlgo::pushValidCentervoxelNeigborsIntoQueue(
    const AccVoxel& centerVoxel, std::queue<AccVoxel>& queue, int ring) {
  std::vector<std::tuple<int, int, int>> neighborRelation =
      generateVoxelNeighbors(ring);

  for (const auto& [dx, dy, dz] : neighborRelation) {
    DGtalPoint3D posTemp(centerVoxel.position[0] + dx,
                         centerVoxel.position[1] + dy,
                         centerVoxel.position[2] + dz);
    auto idTemp = AccumulationSpace::accumulationHash(posTemp);

    if (mapKeychecker(voxelMap, idTemp)) {
      auto& voxelTemp = voxelMap.at(idTemp);
      // skip visied/centre/flag!=0 voxel
      if (!voxelTemp.visited && posTemp != centerVoxel.position &&
          voxelTemp.label == 0) {
        voxelTemp.label = centerVoxel.label;
        queue.push(voxelTemp);
      }
    }
  }
}

// 1) strategy: using the result we computed based on the simple radius
// 2) strategy: starting from the highest accumulation voxel again
// Find the neighbor faces based on the face id(intermediate element is all around vertices)
void RadiusClusterAlgo::markDynamicRadiusAccLabel() {

  std::queue<AccVoxel> localQ;
  // Init the empty cluster(cluster 0 doesn't be used)
  cluster.push_back(std::vector<DGtalUint>());
  clusterLabel = 0;
  if (!accPQ.empty()) {
    accPQ = std::priority_queue<AccVoxel>();
  }
  for (auto v : voxelMap)
    accPQ.push(v.second);
  log->add(LogLevel::INFO, "START: DRA loading global queue size is ",
           accPQ.size());

  // LOOP 1: pop current maximum votes voxel
  while (!accPQ.empty()) {
    auto vCurrent = accPQ.top();
    auto pCurrent = vCurrent.position;
    auto idCurrent = accumulationHash(vCurrent.position);

    // Add a new cluster
    if (voxelMap.at(idCurrent).visited) {
      accPQ.pop();
      continue;
    }
    voxelMap.at(idCurrent).visited = true;
    voxelMap.at(idCurrent).label = ++clusterLabel;
    vCurrent.label = clusterLabel;
    cluster.push_back(std::vector<DGtalUint>());
    cluster[clusterLabel].push_back(idCurrent);

    // For each unvisited center voxel, push all valid neighbors and associated voxels(the incident maximum votes voxel of all associated faces) into the local queue
    int radius =
        static_cast<int>(std::round(getAccumulationAverageRadius(vCurrent))) -
        radiusDepth;
    pushValidCentervoxelNeigborsIntoQueue(vCurrent, localQ, radius);
    pushValidCentervoxelAssociatedVoxelsIntoQueue(vCurrent, localQ, 1);

    // LOOP 2: traverse the candidate voxel of local queue
    while (!localQ.empty()) {
      auto vAssociated = localQ.front();
      auto pAssociated = vAssociated.position;
      auto idAssociated = accumulationHash(vAssociated.position);

      // If the distance between the candidate voxel and the associated voxel is larger than the radius
      auto dis = glm::distance(
          glm::vec3(pAssociated[0], pAssociated[1], pAssociated[2]),
          glm::vec3(pCurrent[0], pCurrent[1], pCurrent[2]));
      if (dis > 1.5 * radius) {
        localQ.pop();
        log->add(LogLevel::INFO, "Remove: outlier [", clusterLabel,
                 "] : ", pAssociated, " because of the distance is ", dis,
                 " and radius is ", radius);
        continue;
      }

      if (voxelMap.at(idAssociated).visited) {
        localQ.pop();
        continue;
      } else if (voxelMap.at(idAssociated).label == 0)
        log->add(LogLevel::ERROR, pAssociated, " is not visited and no label");

      // mark visited, store in a cluster and label rest neighbors
      voxelMap.at(idAssociated).visited = true;
      cluster[clusterLabel].push_back(idAssociated);
      localQ.pop();
    }
  }

  log->add(LogLevel::INFO, "FINISH: Found ", clusterLabel, " clusters");
}

float RadiusClusterAlgo::computerSpaceGradient(const DGtalPoint3D& v) {
  // 获取邻近体素（需处理边界情况）
  auto x_prev = DGtalPoint3D(v[0] - 1, v[1], v[2]);
  auto x_next = DGtalPoint3D(v[0] + 1, v[1], v[2]);
  auto y_prev = DGtalPoint3D(v[0], v[1] - 1, v[2]);
  auto y_next = DGtalPoint3D(v[0], v[1] + 1, v[2]);
  auto z_prev = DGtalPoint3D(v[0], v[1], v[2] - 1);
  auto z_next = DGtalPoint3D(v[0], v[1], v[2] + 1);

  float xp = voxelMap.find(accumulationHash(x_prev)) == voxelMap.end()
                 ? 0
                 : voxelMap.at(accumulationHash(x_prev)).votes;
  float xn = voxelMap.find(accumulationHash(x_next)) == voxelMap.end()
                 ? 0
                 : voxelMap.at(accumulationHash(x_next)).votes;
  float yp = voxelMap.find(accumulationHash(y_prev)) == voxelMap.end()
                 ? 0
                 : voxelMap.at(accumulationHash(y_prev)).votes;
  float yn = voxelMap.find(accumulationHash(y_next)) == voxelMap.end()
                 ? 0
                 : voxelMap.at(accumulationHash(y_next)).votes;
  float zp = voxelMap.find(accumulationHash(z_prev)) == voxelMap.end()
                 ? 0
                 : voxelMap.at(accumulationHash(z_prev)).votes;
  float zn = voxelMap.find(accumulationHash(z_next)) == voxelMap.end()
                 ? 0
                 : voxelMap.at(accumulationHash(z_next)).votes;

  log->add(LogLevel::INFO, "x_prev votes: ", xp);
  log->add(LogLevel::INFO, "x_next votes: ", xn);
  log->add(LogLevel::INFO, "y_prev votes: ", yp);
  log->add(LogLevel::INFO, "y_next votes: ", yn);
  log->add(LogLevel::INFO, "z_prev votes: ", zp);
  log->add(LogLevel::INFO, "z_next votes: ", zn);

  // 计算偏导数
  float delta = 1;
  float dx = (xn - xp) / (2 * delta);
  float dy = (yn - yp) / (2 * delta);
  float dz = (zn - zp) / (2 * delta);

  // 计算梯度模长
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void RadiusClusterAlgo::initializeAdaptivePriorityQueue() {
  if (!global_queue.empty()) {  // clear the global queue
    global_queue =
        std::priority_queue<AccVoxel, std::vector<AccVoxel>, ScoreComparator>();
  }
  for (auto v : voxelMap)
    global_queue.push(v.second);
  log->add(LogLevel::INFO, "START: DRA loading adaptive global queue size is ",
           global_queue.size());
}
void RadiusClusterAlgo::updateAdaptivePriorityQueue() {}

void RadiusClusterAlgo::markAdaptiveRadiusAccLabel() {

  std::queue<AccVoxel> localQ;
  // Init the empty cluster(cluster 0 doesn't be used)
  cluster.push_back(std::vector<DGtalUint>());
  clusterLabel = 0;
  initializeAdaptivePriorityQueue();  // init the global queue

  // LOOP 1: pop current maximum votes voxel
  while (!global_queue.empty()) {
    auto vCurrent = global_queue.top();
    auto pCurrent = vCurrent.position;
    auto idCurrent = accumulationHash(vCurrent.position);

    // Add a new cluster
    if (voxelMap.at(idCurrent).visited) {
      global_queue.pop();
      continue;
    }
    voxelMap.at(idCurrent).visited = true;
    voxelMap.at(idCurrent).label = ++clusterLabel;
    vCurrent.label = clusterLabel;
    cluster.push_back(std::vector<DGtalUint>());
    cluster[clusterLabel].push_back(idCurrent);

    // 此处计算半径
    float grad_norm = computerSpaceGradient(pCurrent);
    float R_max =
        static_cast<int>(std::round(getAccumulationAverageRadius(vCurrent)));
    int radius =
        DEFAULT_RMAX + (R_max - DEFAULT_RMAX) * std::exp(-1.0f * grad_norm);
    //static_cast<int>(std::min(DEFAULT_ALPHA / (grad_norm + 1e-5f), R_max));

    // LOOP 2: traverse the candidate voxel of local queue
    while (!localQ.empty()) {
      auto vAssociated = localQ.front();
      auto pAssociated = vAssociated.position;
      auto idAssociated = accumulationHash(vAssociated.position);

      // If the distance between the candidate voxel and the associated voxel is larger than the radius
      // 此处进行空间近邻检查
      auto dis = glm::distance(
          glm::vec3(pAssociated[0], pAssociated[1], pAssociated[2]),
          glm::vec3(pCurrent[0], pCurrent[1], pCurrent[2]));
      if (dis > radius) {
        localQ.pop();
        log->add(LogLevel::INFO, "Remove: outlier [", clusterLabel,
                 "] : ", pAssociated, " because of the distance is ", dis,
                 " and radius is ", radius);
        continue;
      }

      if (voxelMap.at(idAssociated).visited) {
        localQ.pop();
        continue;
      } else if (voxelMap.at(idAssociated).label == 0)
        log->add(LogLevel::ERROR, pAssociated, " is not visited and no label");

      // 此处添加近邻检查
      pushValidCentervoxelNeigborsIntoQueue(vCurrent, localQ, radius);
      pushValidCentervoxelAssociatedVoxelsIntoQueue(vCurrent, localQ, 1);
      // mark visited, store in a cluster and label rest neighbors
      voxelMap.at(idAssociated).visited = true;
      cluster[clusterLabel].push_back(idAssociated);
      localQ.pop();
    }
    updateAdaptivePriorityQueue();
  }

  // 后处理：聚类合并
  //mergeClusters();
  log->add(LogLevel::INFO, "FINISH: Found ", clusterLabel, " clusters");
}

}  // namespace AccumulationAlgorithms