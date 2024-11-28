#include "AccumulationAlgorithms.h"
#include "Timer.h"

namespace AccumulationAlgorithms {

SimpleCluster::SimpleCluster() : accThreshold(0), confThreshold(0.0) {
  std::cerr << "SimpleCluster default constructor need to be explicit initialized normally" << std::endl;
}
SimpleCluster::SimpleCluster(NormalAccumulationSpace& normalAccSpace, size_t thAcc, double thConf,
                             std::shared_ptr<AccumulationLog> logPtr)
    : log(logPtr), accThreshold(thAcc), confThreshold(thConf) {
  voxelList = normalAccSpace.voxelList;
  // pointList = normalAccSpace.pointList;
  buildCluster(voxelList, thAcc, thConf, logPtr, 1);
}

void SimpleCluster::clearCluster() {
  if (log == nullptr) {
    return;
  }
  mapVoxel.clear();
  accSimpleCluster.clear();
  clusterLabel = 0;
  accPQ = std::priority_queue<AccVoxel>();
  log->add(LogLevel::INFO, "The simple cluster has been cleared");
}

// void SimpleCluster::buildNRningCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
//                                        std::shared_ptr<AccumulationLog> logPtr, int ring) {
//   log = logPtr;
//   log->add(LogLevel::INFO, "Start building simple cluster with accumulation thereshold: ", accThreshold,
//            " and confidence threshold: ", confThreshold);
//   for (auto& v : accList) {
//     auto hashValue = AccumulationSpace::accumulationHash(v.position);
//     mapVoxel[hashValue] = v;
//     // pointList.push_back(GLMPoint3D{v.position[0], v.position[1], v.position[2]});
//   }

//   log->add(LogLevel::INFO, "Simple clustering based on accumulation(value) begins: ");

//   Timer timer("SimpleCluster");
//   timer.start();
//   markNRingAccLabel(mapVoxel, accPQ, clusterLabel, accSimpleCluster, ring);
//   timer.stop();
//   log->add(LogLevel::INFO, timer.log());
//   log->add(LogLevel::INFO, "Finish building simple cluster with ", mapVoxel.size(), " voxels");
// }


void SimpleCluster::buildCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                                 std::shared_ptr<AccumulationLog> logPtr, int ring) {
  log = logPtr;
  log->add(LogLevel::INFO, "Start building simple cluster with accumulation thereshold: ", accThreshold,
           " and confidence threshold: ", confThreshold);
  for (auto& v : accList) {
    auto hashValue = AccumulationSpace::accumulationHash(v.position);
    mapVoxel[hashValue] = v;
    // pointList.push_back(GLMPoint3D{v.position[0], v.position[1], v.position[2]});
  }

  log->add(LogLevel::INFO, "Simple clustering based on accumulation(value) begins: ");

  Timer timer("SimpleCluster");
  timer.start();
  // markAccLabel(mapVoxel, accPQ, clusterLabel, accSimpleCluster);
  markNRingAccLabel(mapVoxel, accPQ, clusterLabel, accSimpleCluster, ring);
  timer.stop();
  log->add(LogLevel::INFO, timer.log());
  log->add(LogLevel::INFO, "Finish building simple cluster with ", mapVoxel.size(), " voxels");
}

void SimpleCluster::buildRadiusCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                                       std::shared_ptr<AccumulationLog> logPtr, polyscope::SurfaceMesh* mesh) {
  log = logPtr;
  log->add(LogLevel::INFO, "Start building simple cluster with accumulation thereshold: ", accThreshold,
           " and confidence threshold: ", confThreshold);
  for (auto& v : accList) {
    auto hashValue = AccumulationSpace::accumulationHash(v.position);
    mapVoxel[hashValue] = v;
    // pointList.push_back(GLMPoint3D{v.position[0], v.position[1], v.position[2]});
  }

  log->add(LogLevel::INFO, "Simple clustering based on accumulation(value) begins: ");

  Timer timer("RadiusCluster");
  timer.start();
  // markAccLabel(mapVoxel, accPQ, clusterLabel, accSimpleCluster);
  markRidiusAccLabel(mapVoxel, accPQ, clusterLabel, accSimpleCluster, mesh);
  timer.stop();
  log->add(LogLevel::INFO, timer.log());
  log->add(LogLevel::INFO, "Finish building simple cluster with ", mapVoxel.size(), " voxels");
}


template <typename MapType, typename KeyType>
bool SimpleCluster::mapKeychecker(const MapType& map, const KeyType& key) {
  auto it = map.find(key);
  if (it != map.end()) return true;
  return false;
}

std::vector<std::tuple<int, int, int>> SimpleCluster::generateNRingNeighbors(int ring) {
  std::vector<std::tuple<int, int, int>> neighbors;
  for (int dx = -ring; dx <= ring; ++dx) {
    for (int dy = -ring; dy <= ring; ++dy) {
      for (int dz = -ring; dz <= ring; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) continue; // exclude center
        // int distance = std::abs(dx) + std::abs(dy) + std::abs(dz);
        // if (distance <= ring) {
        //   neighbors.emplace_back(dx, dy, dz);
        // }
        int distanceSquared = dx * dx + dy * dy + dz * dz;
        if (distanceSquared <= ring * ring) {
          neighbors.emplace_back(dx, dy, dz);
        }
      }
    }
  }
  return neighbors;
}


void SimpleCluster::markNRingNeighbours(const AccVoxel& voxel, HashMap2Voxel& mapVoxel, std::queue<AccVoxel>& queue,
                                        int ring) {
  std::vector<std::tuple<int, int, int>> neighbors = generateNRingNeighbors(ring);

  for (const auto& [dx, dy, dz] : neighbors) {
    DGtalPoint3D posTemp(voxel.position[0] + dx, voxel.position[1] + dy, voxel.position[2] + dz);
    auto idTemp = AccumulationSpace::accumulationHash(posTemp);

    if (mapKeychecker(mapVoxel, idTemp)) {
      auto& voxelTemp = mapVoxel.at(idTemp);
      // skip visied/centre/flag!=0 voxel
      if (!voxelTemp.visited && posTemp != voxel.position && voxelTemp.label == 0) {
        voxelTemp.label = voxel.label;
        queue.push(voxelTemp);
        log->add(LogLevel::DEBUG, "Label in [", voxel.label, "] and push ", voxelTemp.position);
      }
    }
  }
}


void SimpleCluster::markNeighbours(const AccVoxel& voxel, HashMap2Voxel& mapVoxel, std::queue<AccVoxel>& queue) {
  log->add(LogLevel::DEBUG, "Current center is " + std::to_string(voxel.label) + " -> ", voxel.position);
  int gridStep = 1;
  std::vector<std::tuple<int, int, int>> neighbors = {{-gridStep, -gridStep, -gridStep},
                                                      {-gridStep, -gridStep, 0},
                                                      {-gridStep, -gridStep, gridStep},
                                                      {-gridStep, 0, -gridStep},
                                                      {-gridStep, 0, 0},
                                                      {-gridStep, 0, gridStep},
                                                      {-gridStep, gridStep, -gridStep},
                                                      {-gridStep, gridStep, 0},
                                                      {-gridStep, gridStep, gridStep},
                                                      {0, -gridStep, -gridStep},
                                                      {0, -gridStep, 0},
                                                      {0, -gridStep, gridStep},
                                                      {0, 0, -gridStep},
                                                      {0, 0, gridStep},
                                                      {0, gridStep, -gridStep},
                                                      {0, gridStep, 0},
                                                      {0, gridStep, gridStep},
                                                      {gridStep, -gridStep, -gridStep},
                                                      {gridStep, -gridStep, 0},
                                                      {gridStep, -gridStep, gridStep},
                                                      {gridStep, 0, -gridStep},
                                                      {gridStep, 0, 0},
                                                      {gridStep, 0, gridStep},
                                                      {gridStep, gridStep, -gridStep},
                                                      {gridStep, gridStep, 0},
                                                      {gridStep, gridStep, gridStep}};

  for (const auto& [dx, dy, dz] : neighbors) {
    // locate the adjacent voxel id
    DGtalPoint3D posTemp(voxel.position[0] + dx, voxel.position[1] + dy, voxel.position[2] + dz);
    auto idTemp = AccumulationSpace::accumulationHash(posTemp);

    if (mapKeychecker(mapVoxel, idTemp)) {
      const auto& voxelTemp = mapVoxel.at(idTemp);
      // if (isOnBoundary(pAdjacent, bbox)) // skip the boundary voxel
      //   continue;
      if (!voxelTemp.visited && posTemp != voxel.position &&
          voxelTemp.label == 0) // skip the visited voxel and the center
      {
        mapVoxel.at(idTemp).label = voxel.label; // update the current cluster label in global map
        queue.push(mapVoxel.at(idTemp));

        log->add(LogLevel::DEBUG, "Label in [", voxel.label, "] and push ", mapVoxel.at(idTemp).position);
      }
    }
  }
}

template <typename TypePQ>
void SimpleCluster::markAccLabel(HashMap2Voxel& mapVoxel, TypePQ& globalPQ, DGtalUint& clusterLabel,
                                 std::vector<std::vector<DGtalUint>>& cluster) {
  std::queue<AccVoxel> localQ;                 // maintain the local visited:
  cluster.push_back(std::vector<DGtalUint>()); // init the empty cluster 0
  DGtalUint countPush = 0;

  for (auto v : mapVoxel) globalPQ.push(v.second);

  log->add(LogLevel::INFO, "Loaded global queue size is ", globalPQ.size());

  // LOOP I: select the next voxel with the highest votes
  while (!globalPQ.empty()) {
    auto vCurrent = globalPQ.top();
    auto pCurrent = vCurrent.position;
    auto idCurrent = AccumulationSpace::accumulationHash(vCurrent.position);

    if (mapVoxel.at(idCurrent).visited) {
      globalPQ.pop();
      log->add(LogLevel::DEBUG, "Pop(G): ", pCurrent, " and remain ", globalPQ.size());
      continue;
    }

    // LOOP II: traverse the adjacent voxels

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

    markNeighbours(vCurrent, mapVoxel, localQ);
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
      markNeighbours(vAdjacent, mapVoxel, localQ);
    }
  }
  log->add(LogLevel::INFO, "SUCCESS: Have ", cluster.size(), " size");
  log->add(LogLevel::INFO, "SUCCESS: Found ", clusterLabel, " clusters");
}

template <typename TypePQ>
void SimpleCluster::markNRingAccLabel(HashMap2Voxel& mapVoxel, TypePQ& globalPQ, DGtalUint& clusterLabel,
                                      std::vector<std::vector<DGtalUint>>& cluster, int ring) {
  std::queue<AccVoxel> localQ;                 // maintain the local visited:
  cluster.push_back(std::vector<DGtalUint>()); // init the empty cluster 0
  DGtalUint countPush = 0;

  for (auto v : mapVoxel) globalPQ.push(v.second);

  log->add(LogLevel::INFO, "Loaded global queue size is ", globalPQ.size());

  // LOOP I: select the next voxel with the highest votes
  while (!globalPQ.empty()) {
    auto vCurrent = globalPQ.top();
    auto pCurrent = vCurrent.position;
    auto idCurrent = AccumulationSpace::accumulationHash(vCurrent.position);

    if (mapVoxel.at(idCurrent).visited) {
      globalPQ.pop();
      log->add(LogLevel::DEBUG, "Pop(G): ", pCurrent, " and remain ", globalPQ.size());
      continue;
    }

    // LOOP II: traverse the adjacent voxels

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

    // markNeighbours(vCurrent, mapVoxel, localQ);
    markNRingNeighbours(vCurrent, mapVoxel, localQ, ring);
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
      markNRingNeighbours(vAdjacent, mapVoxel, localQ, ring);
    }
  }
  log->add(LogLevel::INFO, "SUCCESS: Have ", cluster.size(), " size");
  log->add(LogLevel::INFO, "SUCCESS: Found ", clusterLabel, " clusters");
}


template <typename TypePQ>
void SimpleCluster::markRidiusAccLabel(HashMap2Voxel& mapVoxel, TypePQ& globalPQ, DGtalUint& clusterLabel,
                                       std::vector<std::vector<DGtalUint>>& cluster, polyscope::SurfaceMesh* mesh) {
  std::queue<AccVoxel> localQ;                 // maintain the local visited:
  cluster.push_back(std::vector<DGtalUint>()); // init the empty cluster 0
  DGtalUint countPush = 0;

  for (auto v : mapVoxel) globalPQ.push(v.second);

  log->add(LogLevel::INFO, "Loaded global queue size is ", globalPQ.size());

  // LOOP I: get a voxel with the current highest votes from the PQ
  while (!globalPQ.empty()) {
    auto vCurrent = globalPQ.top();
    auto pCurrent = vCurrent.position;
    auto idCurrent = AccumulationSpace::accumulationHash(vCurrent.position);

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

    // markRadiusNeighbours(vCurrent, mapVoxel, localQ, mesh);
    int radius = static_cast<int>(std::round(getAverageRadiusByAccumulation(vCurrent, mesh)));
    markNRingNeighbours(vCurrent, mapVoxel, localQ, radius);

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
  log->add(LogLevel::INFO, "SUCCESS: Have ", cluster.size(), " size");
  log->add(LogLevel::INFO, "SUCCESS: Found ", clusterLabel, " clusters");
}
glm::vec3 SimpleCluster::getBarycenterByFaceId(polyscope::SurfaceMesh* mesh, size_t faceId) {
  // convert to the polyscope face index
  auto ID = faceId; // + mesh->vertexPositions.data.size();
  size_t start = mesh->faceIndsStart[ID];
  size_t D = mesh->faceIndsStart[ID + 1] - start;
  glm::vec3 faceCenter{0., 0., 0.};
  for (size_t j = 0; j < D; j++) {
    glm::vec3 pA = mesh->vertexPositions.data[mesh->faceIndsEntries[start + j]];
    faceCenter += pA;
  }
  faceCenter /= 3;
  log->add(LogLevel::DEBUG, "Get face ", ID, "'s barycenter: ", faceCenter.x, " ", faceCenter.y, " ", faceCenter.z);
  return faceCenter;
}
float SimpleCluster::getAverageRadiusByAccumulation(const AccVoxel& voxel, polyscope::SurfaceMesh* mesh) {
  auto voxelPos = voxel.position;
  float radius = 0.1f;
  int count = 0;
  for (auto fid : voxel.associatedFaceIds) {
    // 1. get the face center
    auto faceBarycenter = getBarycenterByFaceId(mesh, fid);
    // 2. get the distance between the face center and the current voxel
    auto distance = glm::distance(faceBarycenter, glm::vec3(voxelPos[0], voxelPos[1], voxelPos[2]));
    log->add(LogLevel::DEBUG, "The ", count++, " = ", distance);
    radius += distance;
  }
  radius = radius / voxel.associatedFaceIds.size();
  log->add(LogLevel::INFO, "Get the average radius of the voxel ", voxelPos[0], " ", voxelPos[1], " ", voxelPos[2],
           " which has ", voxel.associatedFaceIds.size(), " faces and distance is ", radius);
  return radius;
}


void SimpleCluster::markRadiusNeighbours(const AccVoxel& voxel, HashMap2Voxel& mapVoxel, std::queue<AccVoxel>& queue,
                                         polyscope::SurfaceMesh* mesh) {
  float avgRadius = getAverageRadiusByAccumulation(voxel, mesh);
  log->add(LogLevel::INFO, "The ", voxel.associatedFaceIds.size(), " faces average radius is ", avgRadius);

  std::vector<std::tuple<int, int, int>> neighbors = generateNRingNeighbors(1);

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

} // namespace AccumulationAlgorithms