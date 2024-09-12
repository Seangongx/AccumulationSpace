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
#include <boost/concept_archetype.hpp>
#include <cstddef>
#include <iostream>
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
#include "AccVoxel.h"
#include "AccVoxelHelper.h"
#include "CLI11.hpp"
#include "Timer.h"
///////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace DGtal;
///////////////////////////////////////////////////////////////////////////////

/**
 @page polyMeshEdit polyMeshEdit

 @brief  polyMeshEdit tools to edit a mesh (add local noise and remove selected faces). Note that the process relies on
 the halfedge data structure that can fail if the input is not topologically consistant. If you want use other type of
 mesh, you can use meshViewerEdit that is based on the simple soup of triangles process (slower selection process).

 @b Usage:   polyMeshEdit [OPTIONS] 1 [2]


 @b Allowed @b options @b are :

 @code

 Positionals:
   1 TEXT:FILE REQUIRED                  an input mesh file in .obj or .off format.
   2 TEXT:FILE=result.obj                an output mesh file in .obj or .off format.


 Options:
   -h,--help                             Print this help message and exit
   -i,--input TEXT:FILE REQUIRED         an input mesh file in .obj or .off format.
   -o,--output TEXT:FILE=result.obj      an output mesh file in .obj or .off format.
 @endcode

 @b Example:

 @code
    polyMeshEdit $DGtal/examples/samples/bunnyhead.obj  bunnyEdited.obj
 @endcode

 @image html respolyMeshEdit.png "Example of result. "

 @see
 @ref polyMeshEdit.cpp

 */


// =============================================================
// ======================= Definitions =========================
// =============================================================

typedef PolygonalSurface<Z3i::RealPoint> PolySurface;
typedef glm::vec3 PolyPoint;
typedef vector<PolyPoint> PointLists;

static PolySurface currentPolysurf;
static PolySurface firstPolysurf;

/// Surface Editing
static std::vector<int> vectSelection;
static float minPaintRad = 1.0;
static float maxPaintRad = 100.0;
static float minNoiseLevel = 1.0;
static float maxNoiseLevel = 100.0;

static float paintRad = 1.0;
static float noiseLevel = 1.0;
static int partialF = 1;
static int randLarge = 100000;
static const int unselectFlag = 200;
static const int selectFlag = 50;
static const int cursorFlag = 1;

static string outputFileName{"result.obj"};
static string defaultMeshColorQuantityName{"associated faces color"};

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
static PointLists selectingVoxels;
std::vector<double> accmulationScalarValues;
std::vector<glm::vec3> currentfacesColor;

static FaceMap faceMapVoxel;

// global selection data
static size_t clickCount = 0;
static size_t selectedElementId = 0;
static unordered_map<size_t, vector<AccVoxel::Uint>> selectedAssociatedFacesMap;

static string structureSelected{""};
static string promptText = "select nothing at the beginning\n";
static int faceSelectedId = -1;
static int voxelSelectedId = -1;


// structure type
// enum STRUCTURETYPE { MESH, POINTCLOUD };
// unordered_map<string, STRUCTURETYPE> structureTypes;

// =============================================================
// ======================== Functions ==========================
// =============================================================

void initFacesMap() {
  for (auto v : voxelList) {
    for (auto id : v.faces) {
      size_t faceID = id;

      // need to test the coherence with voxelmap
      faceMapVoxel.addValue(id, AccVoxelHelper::accumulationHash(v.p));
    }
  }
  faceMapVoxel.print();
}

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

// Polyscope use glm::vec3 as point type
std::vector<glm::vec3> getPointsFromVoxelist(std::vector<AccVoxel>& voxelList) {
  std::vector<glm::vec3> points;
  for (AccVoxel v : voxelList) {
    points.push_back(glm::vec3(v.p[0], v.p[1], v.p[2]));
  }
  return points;
}

// Visualize accumulation set in space
void addPointCloudInPolyscopeFrom(string structName, PointLists structPoints, double structRadius,
                                  glm::vec3 structColor) {

  polyscope::PointCloud* psCloud = polyscope::registerPointCloud(structName, structPoints);
  psCloud->setPointRadius(structRadius);
  // set accmuluation votes as scalar quantity)
  accmulationScalarValues.resize(voxelList.size());
  for (size_t i = 0; i < structPoints.size(); i++) {
    accmulationScalarValues[i] = voxelList[i].votes;
  }
  // use turbo color map (default)
  std::vector<double> accumulationQuantity(structPoints.size());
  for (size_t i = 0; i < structPoints.size(); i++) {
    accumulationQuantity[i] = accmulationScalarValues[i];
  }
  psCloud->addScalarQuantity("accumulationQuantity", accumulationQuantity)->setColorMap("turbo")->setEnabled(true);
  psCloud->setTransparency(0.8);
  psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);

  {
    // setTransparencyQuantity does not support pick selection yet
    // auto q2 = psCloud->addScalarQuantity("accumulationTransparency", accumulationTransparency);
    // psCloud->setTransparencyQuantity(q2);

    // polyscope::loadColorMap("sampleColorMap", "/home/adam/Desktop/AccumulationSpace/samples/sample_colormap.png");
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
  auto digsurf = polyscope::registerSurfaceMesh("InputMesh", psurf.positions(), faces);
  digsurf->setSurfaceColor(defaultColor);
  digsurf->setTransparency(0.4)->setEnabled(true);
  // updateSelection();
}

static Z3i::RealPoint getFaceBarycenter(const PolySurface& polysurff, const PolySurface::Face& aFace) {
  Z3i::RealPoint res(0.0, 0.0, 0.0);
  for (auto const& v : polysurff.verticesAroundFace(aFace)) {
    res += polysurff.position(v);
  }
  return res / polysurff.verticesAroundFace(aFace).size();
}

// Helper function
std::vector<PolySurface::Face> faceAround(const PolySurface& polysurff, PolySurface::Face faceId, double radius) {
  std::vector<PolySurface::Face> result;
  std::queue<PolySurface::Vertex> q;
  std::map<PolySurface::Face, bool> fVisited;
  std::map<PolySurface::Face, bool> vVisited;

  for (auto const& v : polysurff.verticesAroundFace(faceId)) {
    q.push(v);
  }
  fVisited[faceId] = true;
  bool addNewFaces = true;
  while (!q.empty()) {
    PolySurface::Vertex v = q.front();
    q.pop();
    auto listFace = polysurff.facesAroundVertex(v);
    for (auto const& f : listFace) {
      if (fVisited.count(f) == 0) {
        if ((getFaceBarycenter(polysurff, f) - getFaceBarycenter(polysurff, faceId)).norm() < radius) {
          fVisited[f] = true;
          result.push_back(f);
          for (auto const& v : polysurff.verticesAroundFace(f)) {
            if (vVisited.count(v) == 0) {
              vVisited[v] = true;
              q.push(v);
            }
          }
        }
      }
    }
  }

  return result;
}
/**
 * Select faces from selection with a probability of 1/selFreq
 */
void partialSelect(int selFreq = 1) {
  srand((unsigned)time(NULL));
  for (unsigned int i = 0; i < currentPolysurf.nbFaces(); i++) {
    if (vectSelection[i] == selectFlag) {
      if (rand() % selFreq == 0) {
        vectSelection[i] = unselectFlag;
      } else {
        vectSelection[i] = selectFlag;
      }
    } else {
      vectSelection[i] = unselectFlag;
    }
  }
}

void noisify(double scale = 0.01) {
  srand((unsigned)time(NULL));
  for (unsigned int i = 0; i < currentPolysurf.nbFaces(); i++) {
    Z3i::RealPoint pDep((((double)(rand() % randLarge) - randLarge / 2.0) / randLarge) * scale,
                        (((double)(rand() % randLarge) - randLarge / 2.0) / randLarge) * scale,
                        (((double)(rand() % randLarge) - randLarge / 2.0) / randLarge) * scale);
    if (vectSelection[i] == selectFlag) {
      for (auto f : currentPolysurf.verticesAroundFace(i)) {
        currentPolysurf.positions()[f][0] += pDep[0];
        currentPolysurf.positions()[f][1] += pDep[1];
        currentPolysurf.positions()[f][2] += pDep[2];
      }
    }
  }
  addSurfaceInPolyscopeFrom(currentPolysurf);
}

void deleteSelectedFaces() {
  PolySurface newSur;
  std::vector<bool> vertexUsed(currentPolysurf.nbVertices(), false);
  for (unsigned int i = 0; i < currentPolysurf.nbFaces(); i++) {
    if (vectSelection[i] == unselectFlag) {
      auto face = currentPolysurf.verticesAroundFace(i);
      for (auto v : face) {
        vertexUsed[v] = true;
      }
    }
  }
  auto lp = currentPolysurf.positions();
  for (unsigned int i = 0; i < currentPolysurf.nbVertices(); i++) {
    if (vertexUsed[i]) {
      newSur.addVertex(lp[i]);
    }
  }
  std::vector<int> translateIndexId;
  int currentIndex = 0;
  for (unsigned int i = 0; i < currentPolysurf.nbVertices(); i++) {
    if (vertexUsed[i]) {
      translateIndexId.push_back(currentIndex);
      currentIndex++;
    } else {
      translateIndexId.push_back(-1);
    }
  }
  for (unsigned int f = 0; f < currentPolysurf.nbFaces(); f++) {
    if (vectSelection[f] == unselectFlag) {
      auto face = currentPolysurf.verticesAroundFace(f);
      for (unsigned int i = 0; i < face.size(); i++) {
        face[i] = translateIndexId[face[i]];
      }
      newSur.addPolygonalFace(PolySurface::PolygonalFace(face.cbegin(), face.cend()));
    }
  }
  newSur.build();
  currentPolysurf = newSur;
  addSurfaceInPolyscopeFrom(newSur);
}

void setImguiBegin() {
  srand((unsigned)time(NULL));
  ImGui::Begin("Editing tools");
}

void setImguiEnd() {
  ImGui::Text("%s", promptText.c_str());
  ImGui::End();
}

void setImguiIO(ImGuiIO& io) {
  float dpiScale = 1.8f; // DPI ratio
  io.FontGlobalScale = dpiScale;
}


// void mouseSelectFaces(ImGuiIO& io) {
//   if (io.MouseDoubleClicked[0]) {
//     unsigned long indexSelect = polyscope::pick::getSelection().second;
//     unsigned long nb = 0;
//     // face selected
//     if (indexSelect >= currentPolysurf.nbVertices()) {
//       nb = (unsigned long)polyscope::pick::getSelection().second - currentPolysurf.nbVertices();
//     } else {
//       // vertex selected (selecting a face connected to it)
//       if (currentPolysurf.facesAroundVertex(polyscope::pick::getSelection().second).size() > 0) {
//         nb = currentPolysurf.facesAroundVertex(polyscope::pick::getSelection().second)[0];
//       }
//     }

//     if (nb > 0 && nb < vectSelection.size()) {
//       auto fVois = faceAround(currentPolysurf, nb, paintRad);
//       vectSelection[nb] = cursorFlag;
//       srand((unsigned)time(NULL));
//       for (auto f : fVois) {
//         if (rand() % partialF == 0) {
//           vectSelection[f] = selectFlag;
//         } else {
//           vectSelection[f] = unselectFlag;
//         }
//       }
//     }
//   }
// }

void mouseSelectIndexTest(ImGuiIO& io) {

  std::pair<polyscope::Structure*, size_t> selection = polyscope::pick::getSelection();

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

      auto facePickIndStart = currentPolysurf.nbVertices();
      auto edgePickIndStart = facePickIndStart + currentPolysurf.nbFaces();

      if (faceSelectedId < facePickIndStart) {
        promptText = "[Surface Mesh]Selected Vertex: " + to_string(faceSelectedId);
      } else if (faceSelectedId < edgePickIndStart) {
        (faceSelectedId -= facePickIndStart);
        promptText = "[Surface Mesh]Selected Face: " + to_string(faceSelectedId);
      } else {
        promptText = "[Surface Mesh]Selected unknown element: " + to_string(faceSelectedId);
      }
    }
    // if (selection.first->typeName() == "Point Cloud") {

    //   voxelSelectedId = selection.second;
    //   promptText = "[Point Cloud]Selected Accumulation: " + to_string(voxelSelectedId);
    // }
    /*     else {
          promptText = "what happen?";
        } */
    // selected
  }
  // voxel

  /*     auto voxelHash meshFacesMap.getValues(nb);
      point
          polyscope::PointCloud *pt = polyscope::registerPointCloud("a voxel", selectPoint); */
  /*
      if (faceSelectedId >= 0)
      {
          ImGui::Text("[Surface Mesh]Selected Triangle: %d", faceSelectedId);
      }
      else if (voxelSelectedId >= 0)
      {
          ImGui::Text("[Point Cloud]Selected Accumulation: %d", voxelSelectedId);
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
  vector<AccVoxel::Uint> tempSelectedAssociatedFacesList;
  selectedAssociatedFacesMap[selectedElementId] = voxelList[voxelId].faces;
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
  double tempId = (voxelList[selectedElementId].votes * 1.0l - 21) / 317;
  auto tempColor = usedColorMap.getValue(tempId);
  for (auto faceId : selectedAssociatedFacesMap[selectedElementId]) {
    currentfacesColor[faceId] = tempColor;
  }
  digsurf->addFaceColorQuantity(defaultMeshColorQuantityName, currentfacesColor)->setEnabled(true);
}

void mouseSelectAccumulation(ImGuiIO& io) {
  // polyscope default left-click selection
  if (polyscope::pick::haveSelection()) {
    std::pair<polyscope::Structure*, size_t> selection = polyscope::pick::getSelection();
    size_t nb = 0;
    // if (io.MouseClicked[0]) {
    //   // if double clicked an accumulation voxel
    //   if (selection.first->typeName() == "Point Cloud") {
    //     nb = selection.second;
    //     if (nb >= 0) {
    //       clickCount++;
    //       promptText = "[Click Accumulation]: " + to_string(nb);
    //     }
    //   }
    // }
    if (io.MouseDoubleClicked[0]) {
      // if double clicked an accumulation voxel
      if (selection.first->typeName() == "Point Cloud") {
        nb = selection.second;
        if (nb >= 0) {
          clickCount++;
          promptText = "[Double click Accumulation]: " + to_string(nb);
          selectedElementId = nb;
        }

        storeAllAssociatedFacesInList();
        paintAllAssociatedFaces();
      }
    }
  }
}


// Get a structure map
std::map<std::string, std::unique_ptr<polyscope::Structure>>& getStructureMapCreateIfNeeded(std::string typeName) {
  if (polyscope::state::structures.find(typeName) == polyscope::state::structures.end()) {
    polyscope::state::structures[typeName] = std::map<std::string, std::unique_ptr<polyscope::Structure>>();
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

  ImGui::Text("Setting selection size:");
  ImGui::SliderFloat("radius values", &paintRad, minPaintRad, maxPaintRad, "size = %.3f");
  ImGui::Separator();
  ImGui::Text("Set selection freq:");
  ImGui::SliderInt(" freq (1=select all, 2=select 1over2)", &partialF, 1, 10, "freq = %i");
  ImGui::Separator();
  ImGui::Text("Noise parameters:");
  ImGui::SliderFloat("noise scale", &noiseLevel, minNoiseLevel, maxNoiseLevel, "scale = %f");
  ImGui::Separator();

  ImGui::Text("Action:");
  if (ImGui::Button("Clear selection")) {
    for (auto& i : vectSelection) {
      i = unselectFlag;
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("delete selected faces")) {
    deleteSelectedFaces();
  }
  ImGui::SameLine();
  if (ImGui::Button("noisify selected faces")) {
    noisify(noiseLevel);
  }
  ImGui::Separator();
  ImGui::Text("IO");

  if (ImGui::Button("save in .obj")) {
    std::ofstream obj_stream(outputFileName.c_str());
    MeshHelpers::exportOBJ(obj_stream, currentPolysurf);
  }
  ImGui::SameLine();

  if (ImGui::Button("reload src")) {
    currentPolysurf = firstPolysurf;
    addSurfaceInPolyscopeFrom(currentPolysurf);
  }
  ImGui::Separator();
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

  if (ImGui::Button("Reset selection color")) {
    auto tempMesh = polyscope::getSurfaceMesh("InputMesh");
    currentfacesColor.clear();
    currentfacesColor.resize(tempMesh->nFaces(), tempMesh->getSurfaceColor());
    selectedAssociatedFacesMap.clear();
    tempMesh->addFaceColorQuantity(defaultMeshColorQuantityName, currentfacesColor)->setEnabled(true);
    promptText = "[Surface Mesh]Reset surface color";
  }

  if (ImGui::Button(accBtnPressed2 ? "Hide asscociated accumulations" : "Show asscociated accumulations")) {
    // TODO:
    // check face selected
    // currentCachedPointCloud(accumulation voxel) Update
    accBtnPressed2 = false;
  } else {
    accBtnPressed2 = true;
  }
  ImGui::SameLine();
  if (ImGui::Button(accBtnPressed3 ? "Hide voting faces" : "Show voting faces")) {
    // TODO: load file and show all accumulations at once
    accBtnPressed3 = false;
  } else {
    accBtnPressed3 = true;
  }
  // thrid line
  if (ImGui::Button(accBtnPressed4 ? "Hide incident ray" : "Show incident ray")) {
    // check face selected
    // currentCachedPointCloud(accumulation voxel) updace
    accBtnPressed4 = false;
  } else {
    accBtnPressed4 = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("show associated faces")) {
    // check if exists one selected voxel
    // currentCachedDisplayFaces updated
    accBtnPressed5 = false;
  } else {

    accBtnPressed5 = true;
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
  app.description("polyMeshEdit tool to edit a mesh (add local noise and remove selected faces). Note that the process "
                  "relies on the halfedge data structure that can fail if the input is not topologically consistant. "
                  "If you want use other type of mesh, you can use meshViewerEdit that is based on the simple soup of "
                  "triangles process (slower selection process). \n"
                  " polyMeshEdit $DGtal/examples/samples/bunnyhead.obj  bunnyEdited.obj \n");
  app.add_option("-i,--input,1", inputFileName, "an input mesh file in .obj or .off format.")
      ->required()
      ->check(CLI::ExistingFile);
  app.add_option("-a,--inputAcc,2", inputAccName, "an output mesh file in .obj or .off format.", true);
  app.add_option("-o,--output,3", outputFileName, "an output mesh file in .obj or .off format.", true);

  app.get_formatter()->column_width(40);
  CLI11_PARSE(app, argc, argv);

  // build visualization interface
  polyscope::options::programName = "PolyMeshEdit - (DGtalToolsContrib) " + Timer::now();
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

  // pointCloud structure
  voxelList = AccVoxelHelper::getAccVoxelsFromFile(inputAccName);
  primaryVoxels = getPointsFromVoxelist(voxelList);
  addPointCloudInPolyscopeFrom("Primary Voxels", primaryVoxels, 0.008, glm::vec3(1.0f, 1.0f, 1.0f));
  initFacesMap();

  polyscope::show();

  return 0;
}
