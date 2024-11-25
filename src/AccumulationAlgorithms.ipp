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
  buildCluster(voxelList, thAcc, thConf, logPtr);
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

void SimpleCluster::buildCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                                 std::shared_ptr<AccumulationLog> logPtr) {
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
  markAccLabel(mapVoxel, accPQ, clusterLabel, accSimpleCluster);
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


} // namespace AccumulationAlgorithms