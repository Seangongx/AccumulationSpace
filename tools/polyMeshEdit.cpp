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
#include "Denoise.h"
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
typedef vector<PolyPoint> PointLists;
typedef AccumulationVoxel AccVoxel;

// Default parameters
static DGtal::Mesh<DGtal::Z3i::RealPoint> aMesh(true);
static glm::vec3 defaultColor{0.8f, 0.8f, 0.8f};
static PolySurface currentPolysurf;
static PolySurface firstPolysurf;
const string regMeshName = "Mesh";
const string regAccName = "Accumulation";

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

// Surface Denoise
static ImguiParams imguiParams;

/// Accumulation
static string outputFileName{"result.obj"};
static string defaultMeshColorQuantityName{"associated faces color"};
static vector<AccVoxel> voxelList;
static PointLists primaryVoxels;
std::vector<double> accScalars;

NormalAccumulationSpace nas;

// =============================================================
// ======================== Functions ==========================
// =============================================================
void updateSelection() {
  polyscope::removeStructure("selection");
  auto digsurf = polyscope::getSurfaceMesh("InputMesh");
  digsurf->addFaceScalarQuantity("selection", vectSelection)
      ->setMapRange(pair<double, double>{cursorFlag, unselectFlag});
  digsurf->setAllQuantitiesEnabled(true);
}

void clearPoints() {
  if (!primaryVoxels.empty()) {
    vector<glm::vec3> temp;
    primaryVoxels.swap(temp);
  }
}

// Visualize accumulation set in space
void addPointCloudInPolyscopeFrom(PointLists structPoints,
                                  const string accumulationName,
                                  double structRadius, glm::vec3 structColor) {

  polyscope::PointCloud* psCloud =
      polyscope::registerPointCloud(accumulationName, structPoints);
  psCloud->setPointRadius(structRadius);
  // set accmuluation votes as scalar quantity)
  accScalars.resize(voxelList.size());
  cout << "accumulation size: " << voxelList.size() << endl;
  for (size_t i = 0; i < structPoints.size(); i++) {
    accScalars[i] = voxelList[i].votes;
  }
  // use turbo color map (default)
  vector<double> accQuantity(structPoints.size());
  for (size_t i = 0; i < structPoints.size(); i++) {
    accQuantity[i] = accScalars[i];
  }
  psCloud->addScalarQuantity("accumulationQuantity", accQuantity)
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

void addSurfaceInPolyscopeFrom(PolySurface& psurf, const string& meshName) {
  vector<vector<size_t>> faces;
  vectSelection.clear();
  for (auto& face : psurf.allFaces()) {
    faces.push_back(psurf.verticesAroundFace(face));
    vectSelection.push_back(unselectFlag);
  }
  auto digsurf =
      polyscope::registerSurfaceMesh(meshName, psurf.positions(), faces);
  digsurf->setSurfaceColor(defaultColor);
  digsurf->setTransparency(0.5)->setEnabled(true);
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
  ImGui::Text("%s", imguiParams.promptText.c_str());
  ImGui::End();
}

void setImguiIO(ImGuiIO& io) {
  float dpiScale = 1.5f;  // DPI ratio
  io.FontGlobalScale = dpiScale;
}

// It's interesting to see how many times this functioanlity is called
size_t convertMeshElementIdInPolyscope(size_t elementId) {
  if (elementId < 0) {
    imguiParams.promptText =
        "ERROR: In convertMeshElementIdInPolyscope() element < 0";
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

void mouseEventCallback(ImGuiIO& io) {}

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

  ImGui::Text("Accumulation Filter");
  if (ImGui::DragIntRange2("votes", &imguiParams.accFilterRange.first,
                           &imguiParams.accFilterRange.second, 1,
                           imguiParams.defaultAccRange.first,
                           imguiParams.defaultAccRange.second, "Min: %d ",
                           "Max: %d ")) {
    vector<AccumulationVoxel> filterVoxels;
    vector<GLMPoint3D> filterPoints;
    for (auto& v : nas.voxelList) {
      if (v.votes >= imguiParams.accFilterRange.first &&
          v.votes <= imguiParams.accFilterRange.second) {
        filterPoints.push_back(
            GLMPoint3D{v.position[0], v.position[1], v.position[2]});
        filterVoxels.push_back(v);
      }
    }
    polyscope::removePointCloud("Primary Voxels");
    polyscope::PointCloud* psCloud =
        polyscope::registerPointCloud("Primary Voxels", filterPoints);
    psCloud->setPointRadius(0.008);
    // set accmuluation votes as scalar quantity)
    accScalars.resize(filterVoxels.size());
    for (size_t i = 0; i < filterPoints.size(); i++) {
      accScalars[i] = filterVoxels[i].votes;
    }
    // use turbo color map (default)
    psCloud->addScalarQuantity("accumulationQuantity", accScalars)
        ->setColorMap("turbo")
        ->setEnabled(true);
    psCloud->setTransparency(0.5);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
  }

  ImGui::Separator();
  ImGui::SetNextItemOpen(true, ImGuiCond_Once);
  if (ImGui::TreeNode("Denoise menu")) {
    static int n = 1;
    ImGui::Combo("Files", &n, files, 5);
    ImGui::SameLine();
    if (ImGui::Button("Reload Mesh")) {
      //data_ = nullptr;
      //load_mesh(files[n]);
      //data_ = std::make_shared<MeshViewerData>(mesh_);
      // reset mesh
      DGtal::Mesh<DGtal::Z3i::RealPoint> tMesh(true);
      cout << "Reloading mesh: " << files[n] << endl;
      tMesh << files[n];
      polyscope::removeSurfaceMesh(regMeshName);
      polyscope::removePointCloud(regAccName);
      DGtal::MeshHelpers::mesh2PolygonalSurface(tMesh, currentPolysurf);
      addSurfaceInPolyscopeFrom(currentPolysurf, regMeshName);
      firstPolysurf = currentPolysurf;
      // reset parameters
      imguiParams.reset();
      imguiParams.promptText = "Reload mesh: " + std::string(files[n]);
    }

    ImGui::SliderFloat("lambda", &imguiParams.pLambda, 0.01, 0.8);

    ImGui::SliderInt("Implicit iters", &imguiParams.pImplicitIterations, 1, 10);
    if (ImGui::Button("Implicit Minimal Surf")) {
      //data_->implicit_running_ = !data_->implicit_running_;
      //data_->implicit_iterations_prev_ = data_->implicit_iterations_;
    }

    ImGui::SliderInt("Explicit iters", &imguiParams.pExplicitIterations, 1,
                     1000);
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
    if (ImGui::RadioButton("accumulation", &imguiParams.pNoiseMode, 0)) {}
    ImGui::SameLine();
    if (ImGui::RadioButton("laplacian", &imguiParams.pNoiseMode, 1)) {}
    ImGui::Text("denoise scale: ");
    if (ImGui::RadioButton("local", &imguiParams.pNoiseScale, 0)) {}
    ImGui::SameLine();
    if (ImGui::RadioButton("global", &imguiParams.pNoiseScale, 1)) {}
    if (ImGui::SliderFloat("stride(lambda)", &imguiParams.pLambda,
                           imguiParams.minLambda, imguiParams.maxLambda,
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
  polyscope::options::buildGui = true;

  // read input mesh
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

  // Add mesh
  DGtal::MeshHelpers::mesh2PolygonalSurface(aMesh, currentPolysurf);
  polyscope::state::userCallback = callbackFaceID;
  addSurfaceInPolyscopeFrom(currentPolysurf, regMeshName);
  firstPolysurf = currentPolysurf;
  // Add log
  string logName{"noiselog.txt"};
  auto logStream = make_shared<fstream>(logName, ios::out | ios::trunc);
  AccumulationLog log(logName, LogLevel::INFO, logStream);
  shared_ptr<AccumulationLog> logPtr = std::make_shared<AccumulationLog>(log);
  if (inputAccName.empty()) {
    logPtr->add(LogLevel::INFO,
                "No input accumulation file provided, using default");
  }
  // Build normal accumulation space
  nas.buildFrom(inputAccName, logPtr);
  // pointCloud structure  // Initialize log file
  voxelList = nas.voxelList;
  imguiParams.accFilterRange = nas.rangeVoteValue;
  imguiParams.defaultAccRange = nas.rangeVoteValue;
  // Add accumulation point cloud
  addPointCloudInPolyscopeFrom(nas.pointList, regAccName, 0.008,
                               glm::vec3(1.0f, 1.0f, 1.0f));

  polyscope::show();

  return 0;
}
