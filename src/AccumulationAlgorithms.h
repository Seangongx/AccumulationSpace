#ifndef ACCUMULATIONALGORITHMS_H
#define ACCUMULATIONALGORITHMS_H

/**
 * @file AccumulationAlgorithms.h
 * @brief Define the main data structure and test algorithm for an accumulation
 * space.
 *
 * This file is part of the DGtal library/DGtalTools-contrib Project.
 *
 * @author Bertrand Kerautret (\c bertrand.kerautret@univ-lyon2.fr )
 *         Xun Gong (\c x.gong@univ-lyon2.fr.com)
 * @date 2024-10-08
 */

//////////////////////////////////////////////////////////////////////////////
#include <map>
#include <queue>
#include <vector>
///////////////////////////////////////////////////////////////////////////////
#include "AccumulationSpace.h"
///////////////////////////////////////////////////////////////////////////////
#include "polyscope/surface_mesh.h"

namespace AccumulationAlgorithms {
// Basic data types:
typedef AccumulationSpace::DGtalUint DGtalUint;
typedef AccumulationSpace::DGtalPoint3D DGtalPoint3D;
typedef AccumulationSpace::GLMPoint3D GLMPoint3D;
// Algo data types:
typedef AccumulationSpace::AccumulationVoxel AccVoxel;
typedef AccumulationSpace::NormalAccumulationSpace NormalAccumulationSpace;
typedef AccumulationSpace::LogLevel LogLevel;
typedef AccumulationSpace::AccumulationLog AccumulationLog;
using AccumulationSpace::accumulationHash;
typedef std::map<DGtalUint, AccVoxel> HashMap2Voxel;

class ClusterAlgoBase {
public:
  ClusterAlgoBase(){};
  virtual ~ClusterAlgoBase();
  virtual void clearCluster();

  // public methods:
  template <typename MapType, typename KeyType>
  bool mapKeychecker(const MapType& map, const KeyType& key);
  std::vector<std::tuple<int, int, int>> generateVoxelNeighbors(int ring);
  void markVoxelNeighbours(const AccVoxel& voxel, HashMap2Voxel& voxelMap, std::queue<AccVoxel>& queue, int ring);

  // public members:
  DGtalUint clusterLabel = 0;
  std::vector<AccVoxel> voxelList;
  std::vector<GLMPoint3D> pointList; // store the cluster points for visualization from class outside
  //  <Position Hashvalue -> Voxel Object>
  HashMap2Voxel voxelMap;
  // <Cluster Label -> Position Hashvalue>
  std::vector<std::vector<DGtalUint>> cluster;
  std::priority_queue<AccVoxel> accPQ;

protected:
  std::shared_ptr<AccumulationLog> log;
  size_t accThreshold = 0;
  double confThreshold = 0.0f;
};

class NeighbourClusterAlgo : public ClusterAlgoBase {
public:
  NeighbourClusterAlgo(){};
  NeighbourClusterAlgo(NormalAccumulationSpace& normalAccSpace, size_t thAcc, double thConf,
                       std::shared_ptr<AccumulationLog> logPtr);
  ~NeighbourClusterAlgo() override;

  void clearCluster() override;
  void buildCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                    std::shared_ptr<AccumulationLog> logPtr, int ring);

private:
  template <typename TypePQ>
  void markNRingAccLabel(HashMap2Voxel& voxelMap, TypePQ& globalPQ, DGtalUint& clusterLabel,
                         std::vector<std::vector<DGtalUint>>& cluster, int ring);
};

class RadiusClusterAlgo : public ClusterAlgoBase {
public:
  RadiusClusterAlgo(){};
  RadiusClusterAlgo(NormalAccumulationSpace& normalAccSpace, size_t thAcc, double thConf,
                    std::shared_ptr<AccumulationLog> logPtr);
  ~RadiusClusterAlgo() override;

  void clearCluster() override;
  void buildCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                    std::shared_ptr<AccumulationLog> logPtr, polyscope::SurfaceMesh* mesh);

private:
  void markRadiusNeighbours(const AccVoxel& voxel, HashMap2Voxel& mapVoxel, std::queue<AccVoxel>& queue,
                            polyscope::SurfaceMesh* mesh);
  glm::vec3 getBarycenterByFaceId(polyscope::SurfaceMesh* mesh, size_t faceId);
  float getAverageRadiusByAccumulation(const AccVoxel& voxel, polyscope::SurfaceMesh* mesh);

  template <typename TypePQ>
  void markRidiusAccLabel(HashMap2Voxel& mapVoxel, TypePQ& globalPQ, DGtalUint& clusterLabel,
                          std::vector<std::vector<DGtalUint>>& cluster, polyscope::SurfaceMesh* mesh);
};


} // namespace AccumulationAlgorithms


#endif // ACCUMULATIONSPACE_H