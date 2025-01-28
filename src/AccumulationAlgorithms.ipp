#include "AccumulationAlgorithms.h"
#include "Timer.h"

namespace AccumulationAlgorithms {

void ClusterAlgoBase::clearCluster() {
  voxelMap.clear();
  cluster.clear();
  clusterLabel = 0;
  accPQ = std::priority_queue<AccVoxel>();
  if (log != nullptr) {
    log->add(LogLevel::INFO, typeid(*this).name(), " has been cleaned at ", Timer::now());
  }
}
ClusterAlgoBase::~ClusterAlgoBase() { clearCluster(); }
// Check if the key exists in the map
template <typename MapType, typename KeyType>
bool ClusterAlgoBase::mapKeychecker(const MapType& map, const KeyType& key) {
  auto it = map.find(key);
  if (it != map.end()) return true;
  return false;
}
// Generate neighborhoods based on the ring of neighbors
std::vector<std::tuple<int, int, int>> ClusterAlgoBase::generateVoxelNeighbors(int ring) {
  std::vector<std::tuple<int, int, int>> neighbors;
  for (int dx = -ring; dx <= ring; ++dx) {
    for (int dy = -ring; dy <= ring; ++dy) {
      for (int dz = -ring; dz <= ring; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) continue;
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

void ClusterAlgoBase::markVoxelNeighbours(const AccVoxel& voxel, HashMap2Voxel& voxelMap, std::queue<AccVoxel>& queue,
                                          int ring) {
  std::vector<std::tuple<int, int, int>> neighbors = generateVoxelNeighbors(ring);

  for (const auto& [dx, dy, dz] : neighbors) {
    DGtalPoint3D posTemp(voxel.position[0] + dx, voxel.position[1] + dy, voxel.position[2] + dz);
    auto idTemp = accumulationHash(posTemp);
    // check if the key exists in the map
    if (mapKeychecker(voxelMap, idTemp)) {
      auto& voxelTemp = voxelMap.at(idTemp);
      // skip visied/centre/flag!=0 voxel
      if (!voxelTemp.visited && posTemp != voxel.position && voxelTemp.label == 0) {
        voxelTemp.label = voxel.label;
        queue.push(voxelTemp);
        log->add(LogLevel::DEBUG, "Label in [", voxel.label, "] and push ", voxelTemp.position);
      }
    }
  }
}


void NeighbourClusterAlgo::clearCluster() { ClusterAlgoBase::clearCluster(); }
NeighbourClusterAlgo::~NeighbourClusterAlgo() { clearCluster(); }
NeighbourClusterAlgo::NeighbourClusterAlgo(NormalAccumulationSpace& normalAccSpace, size_t thAcc, double thConf,
                                           std::shared_ptr<AccumulationLog> logPtr) {
  voxelList = normalAccSpace.voxelList;
  // Set initial ring to 1
  buildCluster(voxelList, thAcc, thConf, logPtr, 1);
}
void NeighbourClusterAlgo::buildCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                                        std::shared_ptr<AccumulationLog> logPtr, int ring) {
  log = logPtr;
  for (auto& v : accList) {
    auto hashValue = AccumulationSpace::accumulationHash(v.position);
    voxelMap[hashValue] = v;
  }

  log->add(LogLevel::INFO, "Start building ", typeid(*this).name(), " with accumulation thereshold: ", accThreshold,
           " , confidence threshold: ", confThreshold, " and ring: ", ring);
  Timer timer(typeid(*this).name());
  timer.start();
  markNRingAccLabel(voxelMap, accPQ, clusterLabel, cluster, ring);
  timer.stop();
  log->add(LogLevel::INFO, timer.log());
  log->add(LogLevel::INFO, "Finish building ", typeid(*this).name(), " with ", voxelMap.size(), " voxels");
}

template <typename TypePQ>
void NeighbourClusterAlgo::markNRingAccLabel(HashMap2Voxel& voxelMap, TypePQ& globalPQ, DGtalUint& clusterLabel,
                                             std::vector<std::vector<DGtalUint>>& cluster, int ring) {
  std::queue<AccVoxel> localQ;                 // maintain the local visited:
  cluster.push_back(std::vector<DGtalUint>()); // init the empty cluster 0(cluster 0 doesn't be used)
  DGtalUint countPush = 0;

  for (auto v : voxelMap) globalPQ.push(v.second);
  log->add(LogLevel::INFO, "Loaded global queue size is ", globalPQ.size());

  // LOOP I: select the next voxel with the highest votes
  while (!globalPQ.empty()) {
    auto vCurrent = globalPQ.top();
    auto pCurrent = vCurrent.position;
    auto idCurrent = accumulationHash(vCurrent.position);

    if (voxelMap.at(idCurrent).visited) {
      globalPQ.pop();
      log->add(LogLevel::DEBUG, "Pop(G): ", pCurrent, " and remain ", globalPQ.size());
      continue;
    }

    // LOOP II: traverse the adjacent voxels

    // Form a new cluster, push the first voxel and update the current cluster label
    voxelMap.at(idCurrent).visited = true;
    voxelMap.at(idCurrent).label = ++clusterLabel;
    vCurrent.label = clusterLabel;
    cluster.push_back(std::vector<DGtalUint>());
    cluster[clusterLabel].push_back(idCurrent);

    log->add(LogLevel::DEBUG, "(", ++countPush, ") push ", voxelMap.at(idCurrent).position, " in cluster ",
             clusterLabel, " and set visited");
    log->add(LogLevel::DEBUG, "Current cluster center is ", pCurrent, " -> ", vCurrent.label);
    log->add(LogLevel::DEBUG, "Remain(G): ", globalPQ.size());

    // markNeighbours(vCurrent, mapVoxel, localQ);
    markVoxelNeighbours(vCurrent, voxelMap, localQ, ring);
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
      markVoxelNeighbours(vAdjacent, voxelMap, localQ, ring);
    }
  }
  log->add(LogLevel::INFO, "FINISH: Have ", cluster.size(), " size");
  log->add(LogLevel::INFO, "FINISH: Found ", clusterLabel, " clusters");
}


void RadiusClusterAlgo::clearCluster() { ClusterAlgoBase::clearCluster(); }
RadiusClusterAlgo::~RadiusClusterAlgo() { clearCluster(); }

void RadiusClusterAlgo::buildCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                                     std::shared_ptr<AccumulationLog> logPtr, polyscope::SurfaceMesh* mesh) {
  log = logPtr;
  log->add(LogLevel::INFO, "Start building ", typeid(*this).name(), " with accumulation thereshold: ", accThreshold,
           " and confidence threshold: ", confThreshold);
  for (auto& v : accList) {
    auto hashValue = accumulationHash(v.position);
    voxelMap[hashValue] = v;
  }

  log->add(LogLevel::INFO, "Simple clustering based on accumulation(value) begins: ");

  Timer timer("RadiusCluster");
  timer.start();
  // markAccLabel(mapVoxel, accPQ, clusterLabel, accSimpleCluster);
  markRidiusAccLabel(voxelMap, accPQ, clusterLabel, cluster, mesh);
  timer.stop();
  log->add(LogLevel::INFO, timer.log());
  log->add(LogLevel::INFO, "Finish building simple cluster with ", voxelMap.size(), " voxels");
}

void RadiusClusterAlgo::markRadiusNeighbours(const AccVoxel& voxel, HashMap2Voxel& mapVoxel,
                                             std::queue<AccVoxel>& queue, polyscope::SurfaceMesh* mesh) {
  float avgRadius = getAverageRadiusByAccumulation(voxel, mesh);
  log->add(LogLevel::INFO, "The ", voxel.associatedFaceIds.size(), " faces average radius is ", avgRadius);

  std::vector<std::tuple<int, int, int>> neighbors = generateVoxelNeighbors(1);

  for (const auto& [dx, dy, dz] : neighbors) {
    DGtalPoint3D posTemp(voxel.position[0] + dx, voxel.position[1] + dy, voxel.position[2] + dz);
    auto idTemp = AccumulationSpace::accumulationHash(posTemp);

    if (mapKeychecker(mapVoxel, idTemp)) {
      auto& voxelTemp = mapVoxel.at(idTemp);
      // skip visied/centre/flag!=0 voxel
      if (!voxelTemp.visited && posTemp != voxel.position && voxelTemp.label == 0) {
        float dis = glm::distance(glm::vec3(voxelTemp.position[0], voxelTemp.position[1], voxelTemp.position[2]),
                                  glm::vec3(voxel.position[0], voxel.position[1], voxel.position[2]));
        voxelTemp.label = voxel.label;
        queue.push(voxelTemp);
        log->add(LogLevel::DEBUG, "Label in [", voxel.label, "] and push ", voxelTemp.position);
      }
    }
  }
}
glm::vec3 RadiusClusterAlgo::getBarycenterByFaceId(polyscope::SurfaceMesh* mesh, size_t faceId) {
  // convert to the polyscope face index
  size_t start = mesh->faceIndsStart[faceId];
  size_t D = mesh->faceIndsStart[faceId + 1] - start; // Dimension
  glm::vec3 faceCenter{0., 0., 0.};
  for (size_t j = 0; j < D; j++) {
    faceCenter += mesh->vertexPositions.data[mesh->faceIndsEntries[start + j]];
  }
  faceCenter /= 3;
  log->add(LogLevel::DEBUG, "Get face ", faceId, "'s barycenter: ", faceCenter.x, " ", faceCenter.y, " ", faceCenter.z);
  return faceCenter;
}
float RadiusClusterAlgo::getAverageRadiusByAccumulation(const AccVoxel& voxel, polyscope::SurfaceMesh* mesh) {
  auto voxelPos = voxel.position;
  float radius = 0.1f;
  int count = 0;
  for (auto fid : voxel.associatedFaceIds) {
    // 1. get the face center
    auto faceBarycenter = getBarycenterByFaceId(mesh, fid);
    // 2. get the distance between the face center and the current voxel
    auto distance = glm::distance(faceBarycenter, glm::vec3(voxelPos[0], voxelPos[1], voxelPos[2]));
    // log->add(LogLevel::DEBUG, "The ", count++, " = ", distance);
    radius += distance;
  }
  radius = radius / voxel.associatedFaceIds.size();
  log->add(LogLevel::INFO, "Get the average radius of the voxel ", voxelPos[0], " ", voxelPos[1], " ", voxelPos[2],
           " which has ", voxel.associatedFaceIds.size(), " faces and distance is ", radius);
  return radius;
}
template <typename TypePQ>
void RadiusClusterAlgo::markRidiusAccLabel(HashMap2Voxel& mapVoxel, TypePQ& globalPQ, DGtalUint& clusterLabel,
                                           std::vector<std::vector<DGtalUint>>& cluster, polyscope::SurfaceMesh* mesh) {
  std::queue<AccVoxel> localQ;                 // maintain the local visited:
  cluster.push_back(std::vector<DGtalUint>()); // init the empty cluster 0(cluster 0 doesn't be used)
  DGtalUint countPush = 0;

  for (auto v : mapVoxel) globalPQ.push(v.second);
  log->add(LogLevel::INFO, "Loaded global queue size is ", globalPQ.size());

  // LOOP I: get a voxel with the current highest votes from the PQ
  while (!globalPQ.empty()) {
    auto vCurrent = globalPQ.top();
    auto pCurrent = vCurrent.position;
    auto idCurrent = accumulationHash(vCurrent.position);

    if (mapVoxel.at(idCurrent).visited) {
      globalPQ.pop();
      log->add(LogLevel::DEBUG, "Pop(G): ", pCurrent, " and remain ", globalPQ.size());
      continue;
    }

    // Form a new cluster, push the first voxel and update the current cluster label
    mapVoxel.at(idCurrent).visited = true;
    mapVoxel.at(idCurrent).label = ++clusterLabel;
    vCurrent.label = clusterLabel;
    cluster.push_back(std::vector<DGtalUint>());
    cluster[clusterLabel].push_back(idCurrent);

    log->add(LogLevel::DEBUG, "(", ++countPush, ") push ", mapVoxel.at(idCurrent).position, " in cluster ",
             clusterLabel, " and set visited");
    log->add(LogLevel::DEBUG, "Current cluster center is ", pCurrent, " -> ", vCurrent.label);
    log->add(LogLevel::DEBUG, "Remain(G): ", globalPQ.size());

    // LOOP II: traverse the adjacent voxels

    int radius = static_cast<int>(std::round(getAverageRadiusByAccumulation(vCurrent, mesh)));
    markVoxelNeighbours(vCurrent, mapVoxel, localQ, radius); // static radius
    // markRadiusNeighbours(vCurrent, mapVoxel, localQ, mesh); // dynamic radius

    while (!localQ.empty()) {
      auto vAdjacent = localQ.front();
      auto pAdjacent = vAdjacent.position;
      auto idAdjacent = AccumulationSpace::accumulationHash(vAdjacent.position);

      log->add(LogLevel::DEBUG, "Visiting [", clusterLabel, "] : ", pAdjacent);
      log->add(LogLevel::DEBUG, "Remain(L): ", localQ.size());

      if (mapVoxel.at(idAdjacent).visited) {
        localQ.pop();
        continue;
      }
      if (mapVoxel.at(idAdjacent).label == 0) {
        log->add(LogLevel::ERROR, pAdjacent, " is not visited and no label");
      }

      // mark visited, store in a cluster and label rest neighbors
      mapVoxel.at(idAdjacent).visited = true;
      cluster[clusterLabel].push_back(idAdjacent);
      localQ.pop();
      // markNeighbours(vAdjacent, mapVoxel, localQ);
    }
  }
  log->add(LogLevel::INFO, "FINISH: Have ", cluster.size(), " size");
  log->add(LogLevel::INFO, "FINISH: Found ", clusterLabel, " clusters");
}


} // namespace AccumulationAlgorithms