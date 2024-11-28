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

typedef AccumulationSpace::DGtalUint DGtalUint;
typedef AccumulationSpace::AccumulationVoxel AccVoxel;
typedef AccumulationSpace::NormalAccumulationSpace NormalAccumulationSpace;
typedef AccumulationSpace::AccumulationLog AccumulationLog;
typedef AccumulationSpace::LogLevel LogLevel;
typedef AccumulationSpace::DGtalUint DGtalUint;
typedef AccumulationSpace::DGtalPoint3D DGtalPoint3D;
typedef AccumulationSpace::GLMPoint3D GLMPoint3D;

typedef std::map<DGtalUint, AccVoxel> HashMap2Voxel;

/**
 * @class SimpleAccCluster
 * @brief The simplest clustring algorithm for accumulation voxels \n
 * Input: NormalAccumulationSpace, AccumulationLog, SurfaceMesh \n
 * Parameters: threshold of <accumulation, confidence>, neighbour range <ring>
 */
class SimpleCluster {
public:
  SimpleCluster();
  SimpleCluster(NormalAccumulationSpace& normalAccSpace, size_t thAcc, double thConf,
                std::shared_ptr<AccumulationLog> logPtr);
  ~SimpleCluster(){};

  void clearCluster();
  void buildCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                    std::shared_ptr<AccumulationLog> logPtr, int ring);
  void buildRadiusCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                          std::shared_ptr<AccumulationLog> logPtr, polyscope::SurfaceMesh* mesh);

  template <typename MapType, typename KeyType>
  bool mapKeychecker(const MapType& map, const KeyType& key);

  // public members
  // <Position Hashvalue : Voxel Object>
  HashMap2Voxel mapVoxel;
  // <Cluster Label : Position Hashvalue>
  std::vector<std::vector<DGtalUint>> accSimpleCluster;
  std::vector<AccVoxel> voxelList;
  std::vector<GLMPoint3D> pointList;
  DGtalUint clusterLabel = 0;
  std::priority_queue<AccVoxel> accPQ;

private:
  std::vector<std::tuple<int, int, int>> generateNRingNeighbors(int ring);

  void markNRingNeighbours(const AccVoxel& voxel, HashMap2Voxel& mapVoxel, std::queue<AccVoxel>& queue, int ring);

  /// @brief Push surrounding voxels which are unvisited and no label in local queue
  void markNeighbours(const AccVoxel& voxel, HashMap2Voxel& mapVoxel, std::queue<AccVoxel>& queue);

  /// @brief Push surrounding voxels which are unvisited and no label in local queue according to the center radius
  void markRadiusNeighbours(const AccVoxel& voxel, HashMap2Voxel& mapVoxel, std::queue<AccVoxel>& queue,
                            polyscope::SurfaceMesh* mesh);

  /// @brief Using a breadth-first traversal strategy to label all clusters
  /// @tparam TypePQ
  template <typename TypePQ>
  void markAccLabel(HashMap2Voxel& mapVoxel, TypePQ& globalPQ, DGtalUint& clusterLabel,
                    std::vector<std::vector<DGtalUint>>& cluster);
  template <typename TypePQ>
  void markNRingAccLabel(HashMap2Voxel& mapVoxel, TypePQ& globalPQ, DGtalUint& clusterLabel,
                         std::vector<std::vector<DGtalUint>>& cluster, int ring);
  glm::vec3 getBarycenterByFaceId(polyscope::SurfaceMesh* mesh, size_t faceId);
  float getAverageRadiusByAccumulation(const AccVoxel& voxel, polyscope::SurfaceMesh* mesh);

  template <typename TypePQ>
  void markRidiusAccLabel(HashMap2Voxel& mapVoxel, TypePQ& globalPQ, DGtalUint& clusterLabel,
                          std::vector<std::vector<DGtalUint>>& cluster, polyscope::SurfaceMesh* mesh);

  // private members
  std::shared_ptr<AccumulationLog> log;


  size_t accThreshold = 0;
  double confThreshold = .0f;
};


} // namespace AccumulationAlgorithms


#endif // ACCUMULATIONSPACE_H