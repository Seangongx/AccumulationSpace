#ifndef POLYSCOPE_ENVIRONMENT_H
#define POLYSCOPE_ENVIRONMENT_H

/**
 * @file AccumulationSpace.h
 * @brief Define the main data structure and test algorithm for an accumulation
 * space.
 *
 * This file is part of the DGtal library/DGtalTools-contrib Project.
 *
 * @author Bertrand Kerautret (\c bertrand.kerautret@univ-lyon2.fr )
 *         Xun Gong (\c sean.gong814@gmail.com)
 * @date 2024-10-4
 */

///////////////////////////////////////////////////////////////////////////////
#include <string>
#include <vector>
////////////////////////////////////////////////////////////////////////////////
#include "AccumulationAlgorithms.h"
#include "AccumulationAlgorithms.ipp"
#include "AccumulationSpace.h"
#include "AccumulationSpace.ipp"
#include "DGtal/shapes/Mesh.h"
#include "DGtal/shapes/PolygonalSurface.h"
#include "imgui.h"
////////////////////////////////////////////////////////////////////////////////

namespace PolyscopeEnvironment {

typedef DGtal::PolygonalSurface<DGtal::Z3i::RealPoint> PolySurface;
typedef AccumulationSpace::DGtalUint DGtalUint;
typedef AccumulationSpace::AccumulationLog AccumulationLog;
typedef AccumulationSpace::AccumulationVoxel AccVoxel;
typedef AccumulationSpace::NormalAccumulationSpace NormalAccumulationSpace;
typedef AccumulationSpace::LogLevel LogLevel;
// Type definitions in Polyscope
typedef glm::vec3 PsColor;
typedef glm::vec3 PsPoint3D;
typedef std::vector<PsPoint3D> PointLists;

// Manager class for Polyscope environment settings
class Manager {
 public:
  Manager(const std::string& meshFile, const std::string& accFile,
          std::shared_ptr<AccumulationLog> logFileStream);
  ~Manager();

  void init(const NormalAccumulationSpace& nas);
  void addSurface(PolySurface& psurf);
  void addPointCloud(const std::string& structName, PointLists& structPoints,
                     double structRadius);
  void buildHashMap2Voxels();
  void buildFaceMap2Voxels();

  void callback();
  void setImguiBegin();
  void setImguiEnd();
  void setImguiIO(ImGuiIO& io);
  void setImguiCustomPanel();

  // TODO:
  void update();
  void render();

  PolySurface currentPolysurf;
  PolySurface firstPolysurf;

  std::string promptText;
  PointLists selectedAssociatedPoints;
  std::unordered_map<size_t, std::vector<DGtalUint>> selectedAssociatedFacesMap;
  std::vector<PsColor> currentfacesColor;
  std::vector<PsColor> defaultfacesColor;
  // event state control variables
  bool accBtnPressed0 = false;
  bool accBtnPressed1 = false;
  bool accBtnPressed2 = false;
  bool accBtnPressed3 = false;
  bool accBtnPressed4 = true;
  bool accBtnPressed5 = true;
  bool accBtnPressed6 = true;
  bool accBtnPressed7 = true;

 private:
  // Helper functions
  size_t convertMeshElementIdInPolyscope(size_t elementId);
  void findLoadedAssociatedAccumulationsByFaceId();
  void storeSelectedAssociatedFacesInMap();
  void paintFacesOn(
      std::string& meshName, std::string& quantityName,
      std::unordered_map<size_t, std::vector<DGtalUint>>& faceMap);
  void paintSelectedAssociatedAccumulations();

  void paintCluster(AccumulationAlgorithms::ClusterAlgoBase& base);
  void paintColoredClusterFacesOn(
      std::string meshName, std::string quantityName,
      AccumulationAlgorithms::ClusterAlgoBase& base);
  void paintColoredClusterPointsIn(
      std::string pointCloudName,
      AccumulationAlgorithms::ClusterAlgoBase& base);

  // event functions
  void mouseEventCallback(ImGuiIO& io);
  void mouseSelectStructureEvent(ImGuiIO& io);
  void mouseDragSliderEvent(int& step,
                            AccumulationAlgorithms::ClusterAlgoBase& base);
  void buttonResetSelectedColorFacesEvent();
  void buttonResetSelectedColorVoxelsEvent();

  //------------------------------------------------------------------------//
  // Default setting variables

  // Environment settings
  float defaultImguiDPIRatio{1.25f};  // for high DPI screen compatibility
  size_t clickCount = 0;
  size_t selectedElementId = 0;
  int faceSelectedId = -1;
  int voxelSelectedId = -1;

  // Nameing settings
  std::string defaultRegisteredMeshName{"InputMesh"};
  std::string defaultOutputFileName{"result.obj"};
  std::string defaultColorMap{"turbo"};
  std::string defaultMeshColorQuantityName{"default faces color"};
  std::string associatedMeshColorQuantityName{"associated faces color"};

  // Display settings
  float defaultMeshTransparency{0.4f};
  float defaultPointTransparency{0.8f};
  float defaultPointRadius{0.01f};
  PsColor defaultMeshColor{0.8f, 0.8f, 0.8f};
  std::pair<int, int> defaultAccRange{0, 0};
  std::pair<int, int> imguiAccFilterRange{0, 0};

  // Algorithm settings
  int imguiAlgoStep = 0;
  int imguiAlgoRadius = 1;
  int defaultAlgoMaxStep{5};
  int defaultAlgoMaxRadius{5};
  std::set<size_t> associatedAccumulationIds;

  //------------------------------------------------------------------------//
  // Member variables

  // Algorithm
  // [hashValue, voxelData]
  std::unordered_map<size_t, AccVoxel> globalHashMap;
  // [faceId,voxelId]
  std::unordered_map<size_t, std::vector<DGtalUint>> globalFaceMap;

  //Environment
  std::vector<double> accmulationScalarValues;
  DGtal::Mesh<DGtal::Z3i::RealPoint> defaultMesh{true};
  std::shared_ptr<AccumulationLog> log;
  std::pair<double, double> defaultColorMapRange;
  NormalAccumulationSpace nas;
  AccumulationAlgorithms::ClusterAlgoBase clusterAlgo;
};

}  // namespace PolyscopeEnvironment

#endif  // POLYSCOPE_ENVIRONMENT_H