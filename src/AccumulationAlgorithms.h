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
#include "DGtal/shapes/PolygonalSurface.h"
#include "polyscope/surface_mesh.h"

namespace AccumulationAlgorithms {
using namespace AccumulationSpace;
typedef AccumulationSpace::AccumulationVoxel AccVoxel;
typedef DGtal::PolygonalSurface<DGtal::Z3i::RealPoint> PolySurface;
typedef std::map<DGtalUint, AccVoxel> HashMap2Voxel;
typedef std::map<DGtalUint, std::vector<DGtalUint>> FaceMap2Voxels;

class ClusterAlgoBase {
 public:
  ClusterAlgoBase() {};
  virtual ~ClusterAlgoBase();
  virtual void clearCluster();

  // public methods:
  template <typename MapType, typename KeyType>
  bool mapKeychecker(const MapType& map, const KeyType& key);
  std::vector<std::tuple<int, int, int>> generateVoxelNeighbors(int ring);
  void markVoxelNeighbours(const AccVoxel& voxel, std::queue<AccVoxel>& queue,
                           int ring);

  // public members:
  // store the cluster points for visualization from class outside
  std::vector<GLMPoint3D> pointList;
  std::vector<AccVoxel> voxelList;
  //  <Position Hashvalue -> Voxel Object>: The algorithm query map
  HashMap2Voxel voxelMap;
  std::priority_queue<AccVoxel> accPQ;
  //  <Cluster Label -> Position Hashvalue>: The algorithm output
  std::vector<std::vector<DGtalUint>> cluster;
  DGtalUint clusterLabel = 0;

 protected:
  std::shared_ptr<AccumulationLog> log;
  size_t accThreshold = 0;
  double confThreshold = 0.0f;
};

struct ScoreComparator {
  bool operator()(const AccVoxel a, const AccVoxel b) const {
    // 综合评分 = 累积值 * 空间衰减因子
    return a.votes < b.votes;
  }
};

class NeighbourClusterAlgo : public ClusterAlgoBase {
 public:
  NeighbourClusterAlgo() {};
  NeighbourClusterAlgo(NormalAccumulationSpace& normalAccSpace, size_t thAcc,
                       double thConf, std::shared_ptr<AccumulationLog> logPtr);
  ~NeighbourClusterAlgo() override;

  void clearCluster() override;
  void buildCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                    std::shared_ptr<AccumulationLog> logPtr, int ring);

 private:
  void markNRingAccLabel(int ring);
};

class RadiusClusterAlgo : public ClusterAlgoBase {
 public:
  RadiusClusterAlgo() {};
  RadiusClusterAlgo(NormalAccumulationSpace& normalAccSpace, size_t thAcc,
                    double thConf, std::shared_ptr<AccumulationLog> logPtr);
  ~RadiusClusterAlgo() override;

  void clearCluster() override;
  void buildCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                    std::shared_ptr<AccumulationLog> logPtr,
                    const PolySurface& polySurface, char mode, int radius);

  // <Face ID -> Position Hashvalue>
  FaceMap2Voxels faceMap;
  PolySurface surf;
  int radiusDepth = 0;

  // Default parameters for adaptive radius accumulation segmentation
  float DEFAULT_RMAX = 2.0f;    // 2*delta 单位：体素边长 [1~5]
  float DEFAULT_LAMBDA = 0.3f;  // 平衡累积值与空间分布
  float DEFAULT_ALPHA = 0.8f;   // 动态半径基础系数
  float DEFAULT_TAU_V = 0.25f;  // 25%差异允许
  float DEFAULT_TAU_D = 2.5f;   // 单位：体素边长
  std::priority_queue<AccVoxel, std::vector<AccVoxel>, ScoreComparator>
      global_queue;

 private:
  void markRadiusNeighbours(const AccVoxel& voxel, std::queue<AccVoxel>& queue);
  DGtal::Z3i::RealPoint getFaceBarycenter(const PolySurface::Face& aFace);
  float getAccumulationAverageRadius(const AccVoxel& Voxel);
  std::vector<PolySurface::Face> getNeighborFacesByFaceId(
      PolySurface::Face faceId, size_t ring);
  bool isSharingEdge(PolySurface::Face faceId, PolySurface::Face f);

  void markStaticRadiusAccLabel();

  void markDynamicRadiusAccLabel();

  void markAdaptiveRadiusAccLabel();

  //new methods
  void pushValidCentervoxelNeigborsIntoQueue(const AccVoxel& voxel,
                                             std::queue<AccVoxel>& queue,
                                             int ring);

  void pushValidCentervoxelAssociatedVoxelsIntoQueue(
      const AccVoxel& centerVoxel, std::queue<AccVoxel>& queue, int ring);

  float computerSpaceGradient(const DGtalPoint3D& v);
  void initializeAdaptivePriorityQueue();
  void updateAdaptivePriorityQueue();
};

}  // namespace AccumulationAlgorithms

#endif  // ACCUMULATIONSPACE_H