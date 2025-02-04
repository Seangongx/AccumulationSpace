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
typedef DGtal::PolygonalSurface<DGtal::Z3i::RealPoint> PolySurface;
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
typedef std::map<DGtalUint, std::vector<DGtalUint>> FaceMap2Voxels;

class ClusterAlgoBase {
 public:
  ClusterAlgoBase(){};
  virtual ~ClusterAlgoBase();
  virtual void clearCluster();

  // public methods:
  template <typename MapType, typename KeyType>
  bool mapKeychecker(const MapType& map, const KeyType& key);
  std::vector<std::tuple<int, int, int>> generateVoxelNeighbors(int ring);
  void markVoxelNeighbours(const AccVoxel& voxel, HashMap2Voxel& voxelMap,
                           std::queue<AccVoxel>& queue, int ring);

  // public members:
  DGtalUint clusterLabel = 0;
  std::vector<AccVoxel> voxelList;
  // store the cluster points for visualization from class outside
  std::vector<GLMPoint3D> pointList;
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
  NeighbourClusterAlgo(NormalAccumulationSpace& normalAccSpace, size_t thAcc,
                       double thConf, std::shared_ptr<AccumulationLog> logPtr);
  ~NeighbourClusterAlgo() override;

  void clearCluster() override;
  void buildCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                    std::shared_ptr<AccumulationLog> logPtr, int ring);

 private:
  template <typename TypePQ>
  void markNRingAccLabel(HashMap2Voxel& voxelMap, TypePQ& globalPQ,
                         DGtalUint& clusterLabel,
                         std::vector<std::vector<DGtalUint>>& cluster,
                         int ring);
};

class RadiusClusterAlgo : public ClusterAlgoBase {
 public:
  RadiusClusterAlgo(){};
  RadiusClusterAlgo(NormalAccumulationSpace& normalAccSpace, size_t thAcc,
                    double thConf, std::shared_ptr<AccumulationLog> logPtr);
  ~RadiusClusterAlgo() override;

  void clearCluster() override;
  void buildCluster(std::vector<AccVoxel>& accList, size_t thAcc, double thConf,
                    std::shared_ptr<AccumulationLog> logPtr,
                    const PolySurface& surf, char mode);

  // <Face ID -> Position Hashvalue>
  FaceMap2Voxels faceMap;

 private:
  void markRadiusNeighbours(const AccVoxel& voxel, HashMap2Voxel& voxelMap,
                            std::queue<AccVoxel>& queue,
                            const PolySurface& surf);
  glm::vec3 getBarycenterByFaceId(polyscope::SurfaceMesh* mesh, size_t faceId);
  float getAverageRadiusByAccumulation(const AccVoxel& voxel,
                                       polyscope::SurfaceMesh* mesh);
  DGtal::Z3i::RealPoint getFaceBarycenter(const PolySurface& surf,
                                          const PolySurface::Face& aFace);
  float getAccumulationAverageRadius(const AccVoxel& voxel,
                                     const PolySurface& surf);
  std::vector<PolySurface::Face> getNeighborFacesByFaceId(
      const PolySurface& surf, PolySurface::Face faceId, size_t ring);
  std::queue<AccVoxel> getCenterFacesVoxels(const AccVoxel& centerVoxel,
                                            const PolySurface& surf,
                                            HashMap2Voxel& voxelMap,
                                            FaceMap2Voxels& faceMap);
  bool isSharingEdge(const PolySurface& surf, PolySurface::Face faceId,
                     PolySurface::Face f);

  template <typename TypePQ>
  void markStaticRadiusAccLabel(HashMap2Voxel& voxelMap, TypePQ& globalPQ,
                                DGtalUint& clusterLabel,
                                std::vector<std::vector<DGtalUint>>& cluster,
                                const PolySurface& surf);

  template <typename TypePQ>
  void markDynamicRadiusAccLabel(HashMap2Voxel& voxelMap, TypePQ& globalPQ,
                                 DGtalUint& clusterLabel,
                                 std::vector<std::vector<DGtalUint>>& cluster,
                                 const PolySurface& surf,
                                 FaceMap2Voxels faceMap);
};

}  // namespace AccumulationAlgorithms

#endif  // ACCUMULATIONSPACE_H