#include <cstddef>
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
    log->add(LogLevel::INFO, typeid(*this).name(), " has been cleaned at ",
             Timer::now());
  }
}
ClusterAlgoBase::~ClusterAlgoBase() {
  clearCluster();
}
// Check if the key exists in the map
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
                                          HashMap2Voxel& voxelMap,
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

  log->add(LogLevel::INFO,
           "Neighbour clustering based on accumulation(value) begins: ");

  log->add(LogLevel::INFO, "Start building ", typeid(*this).name(),
           " with accumulation thereshold: ", accThreshold,
           " , confidence threshold: ", confThreshold, " and ring: ", ring);
  Timer timer(typeid(*this).name());
  timer.start();
  markNRingAccLabel(voxelMap, accPQ, clusterLabel, cluster, ring);
  timer.stop();
  log->add(LogLevel::INFO, timer.log());
  log->add(LogLevel::INFO, "Finish building ", typeid(*this).name(), " with ",
           voxelMap.size(), " voxels");
}

template <typename TypePQ>
void NeighbourClusterAlgo::markNRingAccLabel(
    HashMap2Voxel& voxelMap, TypePQ& globalPQ, DGtalUint& clusterLabel,
    std::vector<std::vector<DGtalUint>>& cluster, int ring) {
  std::queue<AccVoxel> localQ;                  // maintain the local visited:
  cluster.push_back(std::vector<DGtalUint>());  // init the empty cluster
                                                // 0(cluster 0 doesn't be used)
  DGtalUint countPush = 0;

  for (auto v : voxelMap)
    globalPQ.push(v.second);
  log->add(LogLevel::INFO, "Loaded global queue size is ", globalPQ.size());

  // LOOP I: select the next voxel with the highest votes
  while (!globalPQ.empty()) {
    auto vCurrent = globalPQ.top();
    auto pCurrent = vCurrent.position;
    auto idCurrent = accumulationHash(vCurrent.position);

    if (voxelMap.at(idCurrent).visited) {
      globalPQ.pop();
      log->add(LogLevel::DEBUG, "Pop(G): ", pCurrent, " and remain ",
               globalPQ.size());
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

void RadiusClusterAlgo::clearCluster() {
  ClusterAlgoBase::clearCluster();
}
RadiusClusterAlgo::~RadiusClusterAlgo() {
  clearCluster();
}

void RadiusClusterAlgo::buildCluster(std::vector<AccVoxel>& accList,
                                     size_t thAcc, double thConf,
                                     std::shared_ptr<AccumulationLog> logPtr,
                                     const PolySurface& surf, char mode) {
  log = logPtr;
  voxelList = accList;
  log->add(LogLevel::INFO, "Start building ", typeid(*this).name(),
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
    log->add(LogLevel::DEBUG, "对于voxel ", voxel.position, " 有 ",
             voxel.associatedFaceIds.size(), " 个面");
    for (auto faceId : voxel.associatedFaceIds) {
      faceMap[faceId].push_back(i);
      log->add(LogLevel::DEBUG, "面 ", faceId, " 有对应的 ", voxel.position);
    }
  }

  log->add(LogLevel::INFO,
           "Radius clustering based on accumulation(value) begins: ");

  Timer timer("RadiusCluster");
  timer.start();

  switch (mode) {
    case 0:
      markStaticRadiusAccLabel(voxelMap, accPQ, clusterLabel, cluster, surf);
      break;
    case 1:
      markDynamicRadiusAccLabel(voxelMap, accPQ, clusterLabel, cluster, surf,
                                faceMap);
      break;
    default:
      log->add(LogLevel::ERROR, "Invalid mode");
  }

  timer.stop();
  log->add(LogLevel::INFO, timer.log());
  log->add(LogLevel::INFO, "Finish building simple cluster with ",
           voxelMap.size(), " voxels");
}

void RadiusClusterAlgo::markRadiusNeighbours(const AccVoxel& voxel,
                                             HashMap2Voxel& voxelMap,
                                             std::queue<AccVoxel>& queue,
                                             const PolySurface& surf) {
  float avgRadius = getAccumulationAverageRadius(voxel, surf);
  log->add(LogLevel::INFO, "The ", voxel.associatedFaceIds.size(),
           " faces average radius is ", avgRadius);

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

// // In Polyscope space
// glm::vec3 RadiusClusterAlgo::getBarycenterByFaceId(polyscope::SurfaceMesh* mesh, size_t faceId) {
//   // convert to the polyscope face index
//   size_t start = mesh->faceIndsStart[faceId];
//   size_t D = mesh->faceIndsStart[faceId + 1] - start; // Dimension
//   glm::vec3 faceCenter{0., 0., 0.};
//   for (size_t j = 0; j < D; j++) {
//     faceCenter += mesh->vertexPositions.data[mesh->faceIndsEntries[start + j]];
//   }
//   faceCenter /= D;
//   log->add(LogLevel::DEBUG, "Get face ", faceId, "'s barycenter: ", faceCenter.x, " ", faceCenter.y, " ",
//   faceCenter.z); return faceCenter;
// }
// float RadiusClusterAlgo::getAverageRadiusByAccumulation(const AccVoxel& voxel, polyscope::SurfaceMesh* mesh) {
//   auto voxelPos = voxel.position;
//   float radius = 0.1f;
//   int count = 0;
//   for (auto fid : voxel.associatedFaceIds) {
//     // 1. get the face center
//     auto faceBarycenter = getBarycenterByFaceId(mesh, fid);
//     // 2. get the distance between the face center and the current voxel
//     auto distance = glm::distance(faceBarycenter, glm::vec3(voxelPos[0], voxelPos[1], voxelPos[2]));
//     // log->add(LogLevel::DEBUG, "The ", count++, " = ", distance);
//     radius += distance;
//   }
//   radius = radius / voxel.associatedFaceIds.size();
//   log->add(LogLevel::INFO, "Get the average radius of the voxel ", voxelPos[0], " ", voxelPos[1], " ", voxelPos[2],
//            " which has ", voxel.associatedFaceIds.size(), " faces and distance is ", radius);
//   return radius;
// }

// In DGtal space
DGtal::Z3i::RealPoint RadiusClusterAlgo::getFaceBarycenter(
    const PolySurface& surf, const PolySurface::Face& aFace) {
  DGtal::Z3i::RealPoint res(0.0, 0.0, 0.0);
  for (auto const& v : surf.verticesAroundFace(aFace)) {
    res += surf.position(v);
  }
  return res / surf.verticesAroundFace(aFace).size();
}
float RadiusClusterAlgo::getAccumulationAverageRadius(const AccVoxel& voxel,
                                                      const PolySurface& surf) {
  auto voxelPos = voxel.position;
  float radius = 0.1f;
  for (auto fid : voxel.associatedFaceIds) {
    // 1. get the face center
    auto faceBarycenter = getFaceBarycenter(surf, fid);
    // 2. get the distance between the face center and the current voxel
    auto distance = glm::distance(
        glm::vec3(faceBarycenter[0], faceBarycenter[1], faceBarycenter[2]),
        glm::vec3(voxelPos[0], voxelPos[1], voxelPos[2]));
    radius += distance;
  }
  radius = radius / voxel.associatedFaceIds.size();
  log->add(LogLevel::DEBUG, "Get the average radius of the voxel ", voxelPos[0],
           " ", voxelPos[1], " ", voxelPos[2], " which has ",
           voxel.associatedFaceIds.size(), " faces and distance is ", radius);
  return radius;
}

template <typename TypePQ>
void RadiusClusterAlgo::markStaticRadiusAccLabel(
    HashMap2Voxel& voxelMap, TypePQ& globalPQ, DGtalUint& clusterLabel,
    std::vector<std::vector<DGtalUint>>& cluster, const PolySurface& surf) {

  std::queue<AccVoxel> localQ;                  // maintain the local visited:
  cluster.push_back(std::vector<DGtalUint>());  // init the empty cluster
                                                // 0(cluster 0 doesn't be used)
  DGtalUint countPush = 0;

  for (auto v : voxelMap)
    globalPQ.push(v.second);
  log->add(LogLevel::INFO, "//////////////////////////////////////////////");
  log->add(LogLevel::INFO, "Loaded global queue size is ", globalPQ.size());

  // LOOP I: get a voxel with the current highest votes from the PQ
  while (!globalPQ.empty()) {
    auto vCurrent = globalPQ.top();
    auto pCurrent = vCurrent.position;
    auto idCurrent = accumulationHash(vCurrent.position);

    if (voxelMap.at(idCurrent).visited) {
      globalPQ.pop();
      log->add(LogLevel::DEBUG, "Pop(G): ", pCurrent, " and remain ",
               globalPQ.size());
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
    log->add(LogLevel::DEBUG, "Remain(G): ", globalPQ.size());

    // LOOP II: traverse the adjacent voxels

    // int radius = static_cast<int>(std::round(getAverageRadiusByAccumulation(vCurrent, mesh)));
    int radius = static_cast<int>(
        std::round(getAccumulationAverageRadius(vCurrent, surf)));

    markVoxelNeighbours(vCurrent, voxelMap, localQ, radius);  // static radius
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
      // markNeighbours(vAdjacent, mapVoxel, localQ);
    }
  }
  log->add(LogLevel::INFO, "FINISH: Have ", cluster.size(), " size");
  log->add(LogLevel::INFO, "FINISH: Found ", clusterLabel, " clusters");
}

// In DGtal space
// Check if the face f is sharing edge with the current faceId
bool RadiusClusterAlgo::isSharingEdge(const PolySurface& surf,
                                      PolySurface::Face faceId,
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
    const PolySurface& surf, PolySurface::Face faceId, size_t ring) {

  std::vector<PolySurface::Face> result;
  std::unordered_set<PolySurface::Face> fVisited;

  // 获取当前面的三个顶点
  for (const auto& v : surf.verticesAroundFace(faceId)) {
    log->add(LogLevel::DEBUG, "处理面 ", faceId, " 的一个顶点 ", v);
    for (const auto& f : surf.facesAroundVertex(v)) {
      if (f != faceId && isSharingEdge(surf, faceId, f)) {
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
std::queue<AccVoxel> RadiusClusterAlgo::getCenterFacesVoxels(
    const AccVoxel& centerVoxel, const PolySurface& surf,
    HashMap2Voxel& voxelMap, FaceMap2Voxels& faceMap) {

  std::queue<AccVoxel> localQ;

  for (auto f : centerVoxel.associatedFaceIds) {
    auto neighborFaces = getNeighborFacesByFaceId(surf, f, 1);
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
        // 去重检查
        localQ.push(itv->second);
      }
    }
  }
  return localQ;
}

// 1) strategy: using the result we computed based on the simple radius
// 2) strategy: starting from the highest accumulation voxel again
// Find the neighbor faces based on the face id(intermediate element is all around vertices)
template <typename TypePQ>
void RadiusClusterAlgo::markDynamicRadiusAccLabel(
    HashMap2Voxel& voxelMap, TypePQ& globalPQ, DGtalUint& clusterLabel,
    std::vector<std::vector<DGtalUint>>& cluster, const PolySurface& surf,
    FaceMap2Voxels faceMap) {

  std::queue<AccVoxel> localQ;
  // Init the empty cluster(cluster 0 doesn't be used)
  cluster.push_back(std::vector<DGtalUint>());
  clusterLabel = 0;
  // Initialize the PQ and cluster
  if (!globalPQ.empty()) {
    globalPQ = std::priority_queue<AccVoxel>();
  }
  for (auto v : voxelMap)
    globalPQ.push(v.second);
  log->add(LogLevel::INFO, "Starting DRA and loading global queue size is ",
           globalPQ.size());

  // 1. In the loop, using priority queue to pop all candidate voxel
  while (!globalPQ.empty()) {
    auto vCurrent = globalPQ.top();
    auto pCurrent = vCurrent.position;
    auto idCurrent = accumulationHash(vCurrent.position);

    // Add a new cluster
    if (voxelMap.at(idCurrent).visited) {
      globalPQ.pop();
      continue;
    }
    voxelMap.at(idCurrent).visited = true;
    voxelMap.at(idCurrent).label = ++clusterLabel;
    vCurrent.label = clusterLabel;
    cluster.push_back(std::vector<DGtalUint>());
    cluster[clusterLabel].push_back(idCurrent);

    // For each unvisited center voxel, store all associated faces' voxel in vector
    int radius = static_cast<int>(
        std::round(getAccumulationAverageRadius(vCurrent, surf)));
    localQ = getCenterFacesVoxels(vCurrent, surf, voxelMap, faceMap);

    // 2. In the local loop, traverse the candidate secondary associated voxels
    while (!localQ.empty()) {
      auto vAssociated = localQ.front();
      auto pAssociated = vAssociated.position;
      auto idAssociated = accumulationHash(vAssociated.position);

      // If the distance between the candidate voxel and the associated voxel is larger than the radius
      if (glm::distance(
              glm::vec3(pAssociated[0], pAssociated[1], pAssociated[2]),
              glm::vec3(pCurrent[0], pCurrent[1], pCurrent[2])) > radius) {
        localQ.pop();
        log->add(LogLevel::INFO, "Removing outlier [", clusterLabel,
                 "] : ", pAssociated);
        continue;
      }
      // 3. For each face, get all shared face neighbor faces' voxel and store in the local vector

      log->add(LogLevel::DEBUG, "Visiting [", clusterLabel,
               "] : ", pAssociated);
      log->add(LogLevel::DEBUG, "Remain(L): ", localQ.size());

      if (voxelMap.at(idAssociated).visited) {
        localQ.pop();
        continue;
      }
      if (voxelMap.at(idAssociated).label == 0)
        log->add(LogLevel::ERROR, pAssociated, " is not visited and no label");

      // mark visited, store in a cluster and label rest neighbors
      voxelMap.at(idAssociated).visited = true;
      cluster[clusterLabel].push_back(idAssociated);
      localQ.pop();
    }
  }

  log->add(LogLevel::INFO, "FINISH: Have ", cluster.size(), " size");
  log->add(LogLevel::INFO, "FINISH: Found ", clusterLabel, " clusters");
}

}  // namespace AccumulationAlgorithms