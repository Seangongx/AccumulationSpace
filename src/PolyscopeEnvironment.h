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
#include "AccumulationSpace.h"
#include "AccumulationSpace.ipp"
#include "DGtal/shapes/Mesh.h"
#include "DGtal/shapes/PolygonalSurface.h"
#include "imgui.h"
////////////////////////////////////////////////////////////////////////////////

namespace PolyscopeEnvironment {

typedef DGtal::PolygonalSurface<DGtal::Z3i::RealPoint> PolySurface;
typedef AccumulationSpace::DGtalUint DGtalUint;
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
  Manager(){};
  Manager(const std::string& meshFile, const std::string& accFile, AccumulationSpace::LogLevel logLevel);
  ~Manager(){};

  void init(const NormalAccumulationSpace& nas);
  void addSurface(PolySurface& psurf);
  void addPointCloud(const std::string& structName, PointLists& structPoints, double structRadius);
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
  bool accBtnPressed4 = false;
  bool accBtnPressed5 = false;

private:
  size_t convertMeshElementIdInPolyscope(size_t elementId);
  void findLoadedAssociatedAccumulationsByFaceId();
  void storeSelectedAssociatedFacesInList();
  void paintFacesOn(std::string& meshName, std::string& quantityName,
                    std::unordered_map<size_t, std::vector<DGtalUint>>& faceMap);
  void paintSelectedAssociatedAccumulations();

  void mouseEventCallback(ImGuiIO& io);
  // Mouse functionalities
  void mouseSelectIndexTest(ImGuiIO& io);
  void mouseSelectStructure(ImGuiIO& io);
  // Button functionalities
  void buttonResetSelectedColorFaces();
  void buttonResetSelectedColorVoxels();

  // Default settings
  std::string defaultRegisteredMeshName{"InputMesh"};
  std::string defaultOutputFileName{"result.obj"};
  std::string defaultMeshColorQuantityName{"default faces color"};
  std::string associatedMeshColorQuantityName{"associated faces color"};
  float defaultImguiDPIRatio{1.8f};
  PsColor defaultMeshColor{0.8f, 0.8f, 0.8f};
  float defaultMeshTransparency{0.4f};
  float defaultPointTransparency{0.8f};
  float defaultPointRadius{0.008f};
  DGtal::Mesh<DGtal::Z3i::RealPoint> defaultMesh{true};

  // Operation settings
  size_t clickCount = 0;
  size_t selectedElementId = 0;
  std::set<size_t> associatedAccumulationIds;
  std::pair<double, double> turboColorMapRange;

  int faceSelectedId = -1;
  int voxelSelectedId = -1;

  // Accumulation display relevant conponents
  std::vector<double> accmulationScalarValues;
  std::unordered_map<size_t, AccVoxel> globalHashMap;
  std::unordered_map<size_t, std::vector<DGtalUint>> globalFaceMap;
  AccumulationSpace::NormalAccumulationSpace nas;
};


} // namespace PolyscopeEnvironment

#endif // POLYSCOPE_ENVIRONMENT_H