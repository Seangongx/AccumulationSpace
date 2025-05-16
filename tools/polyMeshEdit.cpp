/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file
 * @ingroup visualisation
 * @author Bertrand Kerautret (\c bertrand.kerautret@univ-lyon2.fr )
 *         Xun Gong (\c sean.gong814@gmail.com)
 *
 * @date 2024/06/18
 *
 * Extended functionalities for the source file of PolyMeshEdit tool
 *
 * This file is part of the DGtal library/DGtalTools-contrib Project.
 */

///////////////////////////////////////////////////////////////////////////////
#include <sys/stat.h>
#include <boost/concept_archetype.hpp>
#include <cstddef>
#include <cstdio>
#include <iostream>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>
///////////////////////////////////////////////////////////////////////////////
#include "DGtal/base/Common.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/MeshReader.h"
///////////////////////////////////////////////////////////////////////////////
#include "glm/fwd.hpp"
#include "imgui.h"
#include "polyscope/pick.h"
#include "polyscope/point_cloud.h"
#include "polyscope/point_cloud.ipp"
#include "polyscope/polyscope.h"
#include "polyscope/render/color_maps.h"
#include "polyscope/render/engine.h"
#include "polyscope/surface_mesh.h"
///////////////////////////////////////////////////////////////////////////////
#include "AccumulationSpace.h"
#include "AccumulationSpace.ipp"
#include "CLI11.hpp"
#include "Timer.h"
///////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace DGtal;
using namespace AccumulationSpace;
///////////////////////////////////////////////////////////////////////////////

/**
 @page polyAccEdit polyAccEdit

 @brief  polyAccEdit tools to display an accumulation voxel set (Using point
 cloud for this experiement) calculated from a mesh. Click an accumulation voxel
 inside the mesh or a triangle face on the mesh can visualize the associated
 faces or voting accumulations on the fly.

 @b Usage:   polyAccEdit [OPTIONS] 1 2


 @b Allowed @b options @b are :

 @code

 Positionals:
   1 TEXT:FILE REQUIRED                  an input mesh file in .obj or .off
 format. 2 TEXT:FILE EQUIRED                an input accumulation file in .dat
 format.

 Options:
   -h,--help                             Print this help message and exit
   -i,--input TEXT:FILE REQUIRED         an input mesh file in .obj or .off
 format.
 @endcode

 @b Example:

 @code
    polyAccEdit /Samples/xxx.obj /Samples/xxx.dat
 @endcode

 @image html respolyAccEdit.png "Example of result. "

 @see
 @ref polyAccEdit.cpp

 */

// =============================================================
// ======================= Definitions =========================
// =============================================================

typedef PolygonalSurface<Z3i::RealPoint> PolySurface;
typedef glm::vec3 PolyPoint;
typedef std::vector<PolyPoint> PointLists;
typedef AccumulationSpace::AccumulationVoxel AccVoxel;

static PolySurface currentPolysurf;
static PolySurface firstPolysurf;

/// Surface Editing
static std::vector<int> vectSelection;
static float minPaintRad = 1.0;
static float maxPaintRad = 100.0;
static float minNoiseLevel = 1.0;
static float maxNoiseLevel = 100.0;
// Surface denoise parameters
static float pLambda = 0.1;
static float minLambda = 0.0;
static float maxLambda = 1.0;
static int pNoiseMode = 0;
static int pNoiseScale = 0;
static int pImplicitIterations = 1;
static int pExplicitIterations = 1;
//
std::pair<int, int> imguiAccFilterRange;
std::pair<int, int> defaultAccRange;

static float paintRad = 1.0;
static float noiseLevel = 1.0;
static int partialF = 1;
static int randLarge = 100000;
static const int unselectFlag = 200;
static const int selectFlag = 50;
static const int cursorFlag = 1;

static string outputFileName{"result.obj"};
static string defaultMeshColorQuantityName{"associated faces color"};

static char* files[]{"Balls.obj", "Bunny_head.obj", "Cat_head.obj",
                     "David328.obj", "Nefertiti_face.obj"};

// struct DataWrapper {
//   explicit DataWrapper(PolySurface& mesh) : surf(mesh), curvature(mesh) {}

//   PolySurface surf;
//   pmp_pupa::SurfaceCurvature curvature;

//   int explicit_iterations_{400};
//   int explicit_iterations_prev_{400};
//   bool explicit_running_{false};

//   int implicit_iterations_{2};
//   int implicit_iterations_prev_{2};
//   bool implicit_running_{false};

//   float lambda_{0.2};

//   bool boundary_smoothing_{false};
// };

/// Accumulation
// event state control variables
static glm::vec3 defaultColor{0.8f, 0.8f, 0.8f};
static bool accBtnPressed0 = false;
static bool accBtnPressed1 = false;
static bool accBtnPressed2 = false;
static bool accBtnPressed3 = false;
static bool accBtnPressed4 = false;
static bool accBtnPressed5 = false;

// global maintain data
static vector<AccVoxel> voxelList;
static PointLists primaryVoxels;
std::vector<double> accmulationScalarValues;
std::vector<glm::vec3> currentfacesColor;
static unordered_map<size_t, AccVoxel> globalHashMap;
static unordered_map<size_t, vector<DGtalUint>> globalFaceMap;

// global selection data
static size_t clickCount = 0;
static size_t selectedElementId = 0;
static PointLists selectedAssociatedPoints;
static unordered_map<size_t, vector<DGtalUint>> selectedAssociatedFacesMap;
static set<size_t> associatedAccumulationIds;
pair<double, double> turboColorMapRange;

static string structureSelected{""};
static string promptText = "select nothing at the beginning\n";
static int faceSelectedId = -1;
static int voxelSelectedId = -1;

NormalAccumulationSpace nas;

// =============================================================
// ======================== Functions ==========================
// =============================================================

void clearPoints() {
  if (!primaryVoxels.empty()) {
    vector<glm::vec3> temp;
    primaryVoxels.swap(temp);
  }
}

void updateSelection() {
  polyscope::removeStructure("selection");
  auto digsurf = polyscope::getSurfaceMesh("InputMesh");
  digsurf->addFaceScalarQuantity("selection", vectSelection)
      ->setMapRange(std::pair<double, double>{cursorFlag, unselectFlag});
  digsurf->setAllQuantitiesEnabled(true);
}

// // Polyscope use glm::vec3 as point type
// std::vector<glm::vec3> getPointsFromVoxelist(std::vector<AccVoxel>& voxelList) {
//   std::vector<glm::vec3> points;
//   for (AccVoxel v : voxelList) {
//     points.push_back(glm::vec3(v.p[0], v.p[1], v.p[2]));
//   }
//   return points;
// }

// Visualize accumulation set in space
void addPointCloudInPolyscopeFrom(string structName, PointLists structPoints,
                                  double structRadius, glm::vec3 structColor) {

  polyscope::PointCloud* psCloud =
      polyscope::registerPointCloud(structName, structPoints);
  psCloud->setPointRadius(structRadius);
  // set accmuluation votes as scalar quantity)
  accmulationScalarValues.resize(voxelList.size());
  cout << "accumulation size: " << voxelList.size() << endl;
  for (size_t i = 0; i < structPoints.size(); i++) {
    accmulationScalarValues[i] = voxelList[i].votes;
  }
  // use turbo color map (default)
  std::vector<double> accumulationQuantity(structPoints.size());
  for (size_t i = 0; i < structPoints.size(); i++) {
    accumulationQuantity[i] = accmulationScalarValues[i];
  }
  psCloud->addScalarQuantity("accumulationQuantity", accumulationQuantity)
      ->setColorMap("turbo")
      ->setEnabled(true);
  psCloud->setTransparency(0.8);
  psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);

  {
    // setTransparencyQuantity does not support pick selection yet
    // auto q2 = psCloud->addScalarQuantity("accumulationTransparency",
    // accumulationTransparency); psCloud->setTransparencyQuantity(q2);

    // polyscope::loadColorMap("sampleColorMap",
    // "/home/adam/Desktop/AccumulationSpace/samples/sample_colormap.png");
  }
  // structureTypes["InputPoints"] = POINTCLOUD;
}

void addSurfaceInPolyscopeFrom(PolySurface& psurf) {
  std::vector<std::vector<std::size_t>> faces;
  vectSelection.clear();
  for (auto& face : psurf.allFaces()) {
    faces.push_back(psurf.verticesAroundFace(face));
    vectSelection.push_back(unselectFlag);
  }
  auto digsurf =
      polyscope::registerSurfaceMesh("InputMesh", psurf.positions(), faces);
  digsurf->setSurfaceColor(defaultColor);
  digsurf->setTransparency(0.4)->setEnabled(true);
  // updateSelection();
}

static Z3i::RealPoint getFaceBarycenter(const PolySurface& polysurff,
                                        const PolySurface::Face& aFace) {
  Z3i::RealPoint res(0.0, 0.0, 0.0);
  for (auto const& v : polysurff.verticesAroundFace(aFace)) {
    res += polysurff.position(v);
  }
  return res / polysurff.verticesAroundFace(aFace).size();
}

void setImguiBegin() {
  srand((unsigned)time(NULL));
  ImGui::Begin("Editing tools");
}

void setImguiEnd() {
  ImGui::Separator();
  ImGui::Text("Debug Information:");
  ImGui::Text("%s", promptText.c_str());
  ImGui::End();
}

void setImguiIO(ImGuiIO& io) {
  //float dpiScale = 1.8f;  // DPI ratio
  //io.FontGlobalScale = dpiScale;
}

// It's interesting to see how many times this functioanlity is called
size_t convertMeshElementIdInPolyscope(size_t elementId) {
  if (elementId < 0) {
    promptText = "ERROR: In convertMeshElementIdInPolyscope() element < 0";
    cerr << "ERROR: In convertMeshElementIdInPolyscope() element < 0" << endl;
    return static_cast<size_t>(-1);
  }
  auto facePickIndStart = currentPolysurf.nbVertices();
  auto edgePickIndStart = facePickIndStart + currentPolysurf.nbFaces();
  if (elementId < facePickIndStart) {         // vertex
  } else if (elementId < edgePickIndStart) {  // face
    elementId -= facePickIndStart;
  } else {  // unknown
    return static_cast<size_t>(-1);
  }
  return elementId;
}

void mouseSelectIndexTest(ImGuiIO& io) {

  std::pair<polyscope::Structure*, size_t> selection =
      polyscope::pick::getSelection();

  if (polyscope::pick::haveSelection()) {
    size_t nb = 0;
    // face selected
    if (selection.first == nullptr) {
      clickCount++;
      promptText = "[" + to_string(clickCount) + "] nothing selected";
      return;
    }
    if (selection.first->typeName() == "Surface Mesh") {
      faceSelectedId = selection.second;
      convertMeshElementIdInPolyscope(faceSelectedId);
    }
    // if (selection.first->typeName() == "Point Cloud") {

    //   voxelSelectedId = selection.second;
    //   promptText = "[Point Cloud]Selected Accumulation: " +
    //   to_string(voxelSelectedId);
    // }
    /*     else {
          promptText = "what happen?";
        } */
    // selected
  }
  // voxel

  /*     auto voxelHash meshFacesMap.getValues(nb);
      point
          polyscope::PointCloud *pt = polyscope::registerPointCloud("a voxel",
     selectPoint); */
  /*
      if (faceSelectedId >= 0)
      {
          ImGui::Text("[Surface Mesh]Selected Triangle: %d", faceSelectedId);
      }
      else if (voxelSelectedId >= 0)
      {
          ImGui::Text("[Point Cloud]Selected Accumulation: %d",
     voxelSelectedId);
      }
      else
      {
          ImGui::Text("No element selected");
      } */
}

void storeAllAssociatedFacesInList() {
  auto voxelId = selectedElementId;
  if (voxelId < 0 || static_cast<size_t>(voxelId) >= voxelList.size()) {
    return;
  }
  selectedAssociatedFacesMap[selectedElementId] =
      voxelList[voxelId].associatedFaceIds;
}

void paintAllAssociatedFaces() {
  // find all associated faces[DGtal face Id] on polyscope mesh
  auto digsurf = polyscope::getSurfaceMesh("InputMesh");
  if (digsurf == nullptr) {
    return;
  }
  // TODO: make function flexible to use different color map
  currentfacesColor.resize(digsurf->nFaces(), digsurf->getSurfaceColor());
  auto& usedColorMap = polyscope::render::engine->getColorMap("turbo");
  auto tempMinMax = getMinMaxVotesCountFrom(voxelList);
  double tempValue =
      (voxelList[selectedElementId].votes * 1.0l - tempMinMax.first) /
      tempMinMax.second;
  auto tempColor = usedColorMap.getValue(tempValue);

  for (auto faceId : selectedAssociatedFacesMap[selectedElementId]) {
    currentfacesColor[faceId] = tempColor;
  }
  digsurf->addFaceColorQuantity(defaultMeshColorQuantityName, currentfacesColor)
      ->setEnabled(true);
}

void findLoadedAssociatedAccumulationsByFaceId() {
  auto faceId = selectedElementId;
  auto it = globalFaceMap.find(faceId);
  if (faceId < 0 || it == globalFaceMap.end()) {
    promptText = "ERROR: Current face " + to_string(selectedElementId) +
                 " in findLoadedAssociatedAccumulationsByFaceId() faceId < 0";
    return;
  }
  for (auto l : it->second) {
    associatedAccumulationIds.insert(l);
  }
}

void paintSelectedAssociatedAccumulations() {
  // find all associated voxels[polyscope vector Id] in polyscope point cloud
  auto primaryPointCloud = polyscope::getPointCloud("Primary Voxels");
  if (primaryPointCloud == nullptr) {
    return;
  }
  primaryPointCloud->setEnabled(false);

  selectedAssociatedPoints.clear();
  vector<double> vScalar;
  for (auto accId : associatedAccumulationIds) {
    selectedAssociatedPoints.push_back(primaryVoxels[accId]);
    vScalar.push_back(accmulationScalarValues[accId]);
    cout << accId << " face and accumulation is "
         << accmulationScalarValues[accId] << endl;
  }

  cout << associatedAccumulationIds.size() << endl;
  // paint selected voxels
  polyscope::PointCloud* tempPointCloud = polyscope::registerPointCloud(
      "Selected Associated Voxels", selectedAssociatedPoints);
  tempPointCloud->setPointRadius(0.008);
  turboColorMapRange = getMinMaxVotesCountFrom(voxelList);
  auto q1 = tempPointCloud->addScalarQuantity("AssoAcc", vScalar);
  q1->setColorMap("turbo");
  q1->setMapRange(turboColorMapRange);
  q1->setEnabled(true);

  tempPointCloud->setTransparency(0.8);
  tempPointCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
  tempPointCloud->setEnabled(true);
}

void mouseSelectAccumulation(ImGuiIO& io) {
  // polyscope default left-click selection
  if (polyscope::pick::haveSelection()) {
    std::pair<polyscope::Structure*, size_t> selection =
        polyscope::pick::getSelection();
    // if double clicked an accumulation voxel
    if (io.MouseDoubleClicked[0]) {
      if (selection.second < 0) {
        promptText = "ERROR: In mouseSelectAccumulation() selection.second < 0";
        cerr << "ERROR: In mouseSelectAccumulation() selection.second < 0"
             << endl;
        return;
      }
      if (selection.first->typeName() == "Point Cloud") {
        clickCount++;
        selectedElementId = selection.second;
        storeAllAssociatedFacesInList();
        paintAllAssociatedFaces();
      } else if (selection.first->typeName() == "Surface Mesh") {
        clickCount++;
        selectedElementId = convertMeshElementIdInPolyscope(selection.second);
        // TODO: find all associated accumulations (only add one each step for
        // now)
        findLoadedAssociatedAccumulationsByFaceId();
        if (associatedAccumulationIds.empty()) {
          cerr << "ERROR: Find a " << associatedAccumulationIds.size() << " map"
               << endl;
          return;
        }
        paintSelectedAssociatedAccumulations();  // one voxel for now
                                                 // vector<accId>
      }
      promptText = "[Double click " + selection.first->typeName() +
                   "]: " + to_string(selectedElementId);
    }
  }
}

void buildHashMap2Voxels() {
  for (auto voxel : voxelList) {
    globalHashMap[accumulationHash(voxel.position)] = voxel;
  }
  cout << "HashMap Finished size: " << globalHashMap.size() << endl;
}

void buildFaceMap2Voxels() {
  for (size_t i = 0; i < voxelList.size(); i++) {
    const auto& voxel = voxelList[i];
    for (auto faceId : voxel.associatedFaceIds) {
      globalFaceMap[faceId].push_back(i);
    }
  }
  cout << "Face Finished size: " << globalFaceMap.size() << endl;
}

// Get a structure map
std::map<std::string, std::unique_ptr<polyscope::Structure>>&
getStructureMapCreateIfNeeded(std::string typeName) {
  if (polyscope::state::structures.find(typeName) ==
      polyscope::state::structures.end()) {
    polyscope::state::structures[typeName] =
        std::map<std::string, std::unique_ptr<polyscope::Structure>>();
  }
  return polyscope::state::structures[typeName];
}

void mouseEventCallback(ImGuiIO& io) {

  // mouseSelectFaces(io)
  mouseSelectIndexTest(io);
  mouseSelectAccumulation(io);
}

void setImguiCustomPanel() {
  // panel settings (spacing and inner margins)
  ImGuiStyle& style = ImGui::GetStyle();
  style.WindowPadding = ImVec2(10, 10);
  style.FramePadding = ImVec2(5, 5);
  style.ItemSpacing = ImVec2(10, 10);

  ImGui::Text("Polyscope interface:");

  if (ImGui::Button("show ")) {
    polyscope::options::buildGui = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("hide")) {
    polyscope::options::buildGui = false;
  }

  // add accumulation operations
  ImGui::Separator();
  ImGui::Text("Objects manipulation");

  if (ImGui::Button("Reset selected color faces")) {
    auto tempMesh = polyscope::getSurfaceMesh("InputMesh");
    if (tempMesh != nullptr) {
      tempMesh->setEnabled(false);
    }
    currentfacesColor.clear();
    currentfacesColor.resize(tempMesh->nFaces(), tempMesh->getSurfaceColor());
    selectedAssociatedFacesMap.clear();
    tempMesh->addFaceColorQuantity(defaultMeshColorQuantityName,
                                   currentfacesColor);
    tempMesh->setEnabled(true);
    promptText = "[" + tempMesh->typeName() + "] Reset faces color";
  }

  if (ImGui::Button("Reset selection color voxels")) {
    auto tempVoxels = polyscope::getPointCloud("Selected Associated Voxels");
    if (tempVoxels != nullptr) {
      selectedAssociatedPoints.clear();
      associatedAccumulationIds.clear();
    }
    tempVoxels = polyscope::registerPointCloud("Selected Associated Voxels",
                                               selectedAssociatedPoints);
    promptText = "[" + tempVoxels->typeName() + "] Reset voxels color";
  }

  ImGui::Text("Accumulation Filter");
  if (ImGui::DragIntRange2("votes", &imguiAccFilterRange.first,
                           &imguiAccFilterRange.second, 1,
                           defaultAccRange.first, defaultAccRange.second,
                           "Min: %d ", "Max: %d ")) {
    std::vector<AccumulationSpace::AccumulationVoxel> filterVoxels;
    std::vector<AccumulationSpace::GLMPoint3D> filterPoints;
    for (auto& v : nas.voxelList) {
      if (v.votes >= imguiAccFilterRange.first &&
          v.votes <= imguiAccFilterRange.second) {
        filterPoints.push_back(AccumulationSpace::GLMPoint3D{
            v.position[0], v.position[1], v.position[2]});
        filterVoxels.push_back(v);
      }
    }
    polyscope::removePointCloud("Primary Voxels");
    polyscope::PointCloud* psCloud =
        polyscope::registerPointCloud("Primary Voxels", filterPoints);
    psCloud->setPointRadius(0.008);
    // set accmuluation votes as scalar quantity)
    accmulationScalarValues.resize(filterVoxels.size());
    for (size_t i = 0; i < filterPoints.size(); i++) {
      accmulationScalarValues[i] = filterVoxels[i].votes;
    }
    // use turbo color map (default)
    psCloud->addScalarQuantity("accumulationQuantity", accmulationScalarValues)
        ->setColorMap("turbo")
        ->setEnabled(true);
    psCloud->setTransparency(0.5);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
  }

  ImGui::Separator();

  if (ImGui::TreeNode("Denoise menu")) {
    static int n = 1;
    ImGui::Combo("Files", &n, files, 5);
    ImGui::SameLine();
    if (ImGui::Button("Reload Mesh")) {
      //data_ = nullptr;
      //load_mesh(files[n]);
      //data_ = std::make_shared<MeshViewerData>(mesh_);
    }

    ImGui::SliderFloat("lambda", &pLambda, 0.01, 0.8);

    ImGui::SliderInt("Implicit iters", &pImplicitIterations, 1, 10);
    if (ImGui::Button("Implicit Minimal Surf")) {
      //data_->implicit_running_ = !data_->implicit_running_;
      //data_->implicit_iterations_prev_ = data_->implicit_iterations_;
    }

    ImGui::SliderInt("Explicit iters", &pExplicitIterations, 1, 1000);
    if (ImGui::Button("Explicit Minimal Surf")) {
      //data_->explicit_running_ = !data_->explicit_running_;
      //data_->explicit_iterations_prev_ = data_->explicit_iterations_;
    }
    //ImGui::SameLine();
    //ImGui::Checkbox("Boundary Smoothing", &data_->boundary_smoothing_);

    if (ImGui::Button("Update Gaussian Curvature")) {
      //set_draw_mode("Texture");
      //auto& k = data_->curvature_.update_gauss_curvature();
      //curvature_to_texcoord(k, data_->curvature_.max_gauss_curvature());

      //update_mesh();
    }
    ImGui::SameLine();

    if (ImGui::Button("Update Mean Curvature")) {
      // set_draw_mode("Texture");
      // auto& k = data_->curvature_.update_mean_curvature();
      // curvature_to_texcoord(k, data_->curvature_.max_mean_curvature());
      // update_mesh();
    }

    ImGui::Text("denoise mode: ");
    if (ImGui::RadioButton("accumulation", &pNoiseMode, 0)) {}
    ImGui::SameLine();
    if (ImGui::RadioButton("laplacian", &pNoiseMode, 1)) {}
    ImGui::Text("denoise scale: ");
    if (ImGui::RadioButton("local", &pNoiseScale, 0)) {}
    ImGui::SameLine();
    if (ImGui::RadioButton("global", &pNoiseScale, 1)) {}
    if (ImGui::SliderFloat("stride(lambda)", &pLambda, minLambda, maxLambda,
                           "%.1f")) {}
    ImGui::TreePop();
  }
}

void callbackFaceID() {
  setImguiBegin();
  setImguiCustomPanel();
  // io
  ImGuiIO& io = ImGui::GetIO();
  setImguiIO(io);
  mouseEventCallback(io);
  setImguiEnd();
  // updateSelection();
}

int main(int argc, char** argv) {

  std::string inputFileName{""};
  std::string inputAccName{""};

  // parse command line using CLI ----------------------------------------------
  CLI::App app;
  app.description(
      "polyAccEdit tools to display an accumulation voxel set (Using point "
      "cloud for this experiement) calculated from a mesh. Click an "
      "accumulation voxel inside the mesh or a triangle face on the mesh can "
      "visualize the associated faces or voting accumulations on the fly."
      "polyAccEdit /Samples/xxx.obj /Samples/xxx.dat \n");
  app.add_option("-i,--input,1", inputFileName,
                 "an input mesh file in .obj or .off format.")
      ->required()
      ->check(CLI::ExistingFile);
  app.add_option("-a,--inputAcc,2", inputAccName,
                 "an input accumulation file in .dat format.", true);

  app.get_formatter()->column_width(40);
  CLI11_PARSE(app, argc, argv);

  // build visualization interface
  polyscope::options::programName =
      "PolyMeshEdit - (DGtalToolsContrib) " + Timer::now();
  polyscope::init();
  // polyscope::view::windowWidth = 1024;
  // polyscope::view::windowHeight = 768;
  polyscope::options::buildGui = true;

  // ImGui::SetWindowFontScale(2.0f);

  // read input mesh
  DGtal::Mesh<DGtal::Z3i::RealPoint> aMesh(true);
  aMesh << inputFileName;
  aMesh.removeIsolatedVertices();
  auto bb = aMesh.getBoundingBox();

  // Setting scale mesh dependant parameters
  minPaintRad = (bb.second - bb.first).norm() / 1000.0;
  maxPaintRad = (bb.second - bb.first).norm() / 2.0;
  minNoiseLevel = (bb.second - bb.first).norm() / 10000.0;
  maxNoiseLevel = (bb.second - bb.first).norm() / 100.0;
  noiseLevel = (bb.second - bb.first).norm() / 1000.0;
  paintRad = (bb.second - bb.first).norm() / 50.0;

  // mesh structure
  DGtal::MeshHelpers::mesh2PolygonalSurface(aMesh, currentPolysurf);
  polyscope::state::userCallback = callbackFaceID;
  addSurfaceInPolyscopeFrom(currentPolysurf);
  firstPolysurf = currentPolysurf;

  std::string defaultLogFile{"noiselog.txt"};
  auto globalLogFileStream = std::make_shared<std::fstream>(
      defaultLogFile, std::ios::out | std::ios::trunc);
  AccumulationLog defaultLog(defaultLogFile, LogLevel::INFO,
                             globalLogFileStream);
  std::shared_ptr<AccumulationSpace::AccumulationLog> defaultLogPtr =
      std::make_shared<AccumulationSpace::AccumulationLog>(defaultLog);

  if (inputAccName.empty()) {
    defaultLogPtr->add(LogLevel::INFO,
                       "No input accumulation file provided, using default");
  }

  nas.buildFrom(inputAccName, defaultLogPtr);
  // pointCloud structure  // Initialize log file
  voxelList = nas.voxelList;
  imguiAccFilterRange = nas.rangeVoteValue;
  defaultAccRange = nas.rangeVoteValue;
  addPointCloudInPolyscopeFrom("Primary Voxels", nas.pointList, 0.008,
                               glm::vec3(1.0f, 1.0f, 1.0f));

  // build necessary data structure
  buildHashMap2Voxels();
  buildFaceMap2Voxels();

  polyscope::show();

  return 0;
}
