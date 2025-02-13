#include <memory>
///////////////////////////////////////////////////////////////////////////////
#include "DGtal/io/readers/MeshReader.h"
#include "DGtal/shapes/MeshHelpers.h"
#include "PolyscopeEnvironment.h"
#include "polyscope/pick.h"
#include "polyscope/point_cloud.h"
#include "polyscope/point_cloud.ipp"
#include "polyscope/polyscope.h"
#include "polyscope/render/color_maps.h"
#include "polyscope/render/engine.h"
#include "polyscope/surface_mesh.h"

namespace PolyscopeEnvironment {

Manager::Manager(const std::string& meshFile, const std::string& accFile,
                 std::shared_ptr<AccumulationLog> logPtr) {
  log = logPtr;
  nas = std::move(NormalAccumulationSpace(accFile, log));
  log->add(LogLevel::INFO, "PolyscopeEnvironment startup ", Timer::now());
  // Initialize polyscope
  polyscope::options::programName =
      "PolyAccEditAlgo - (DGtalToolsContrib) " + Timer::now();
  polyscope::init();
  // polyscope::view::windowWidth = 1024;
  // polyscope::view::windowHeight = 768;
  polyscope::options::buildGui = true;

  // read input mesh
  defaultMesh << meshFile;
  defaultMesh.removeIsolatedVertices();
  auto bb = defaultMesh.getBoundingBox();
  DGtal::MeshHelpers::mesh2PolygonalSurface(defaultMesh, currentPolysurf);

  // Load default data
  addSurface(currentPolysurf);
  firstPolysurf = currentPolysurf;
  addPointCloud("Primary Voxels", nas.pointList, defaultPointRadius);

  promptText = "Select nothing at the beginning\n";
  // build necessary data structure
  buildHashMap2Voxels();
  buildFaceMap2Voxels();
  paintFacesOn(defaultRegisteredMeshName, defaultMeshColorQuantityName,
               globalFaceMap);

  polyscope::state::userCallback = [this]() {
    this->callback();
  };
  polyscope::show();
}
// Visualize accumulation set in space
void Manager::addPointCloud(const std::string& structName,
                            PointLists& structPoints, double structRadius) {
  if (nas.voxelList.size() != nas.pointList.size()) {
    log->add(LogLevel::ERROR, "Error: voxelList and pointList size mismatch");
    return;
  }

  polyscope::PointCloud* psCloud =
      polyscope::registerPointCloud(structName, structPoints);
  psCloud->setPointRadius(structRadius);
  // set accmuluation votes as scalar quantity)
  accmulationScalarValues.resize(nas.voxelList.size());
  for (size_t i = 0; i < structPoints.size(); i++) {
    accmulationScalarValues[i] = nas.voxelList[i].votes;
  }
  // use turbo color map (default)
  psCloud->addScalarQuantity("accumulationQuantity", accmulationScalarValues)
      ->setColorMap("turbo")
      ->setEnabled(true);
  psCloud->setTransparency(defaultPointTransparency);
  psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);

  {
    // polyscope::loadColorMap("sampleColorMap",
    // "AccumulationSpace/samples/sample_colormap.png");
  }
}
Manager::~Manager() {
  std::cout << "PolyscopeEnvironment shutdown " << Timer::now() << std::endl;
  log->add(LogLevel::INFO, "PolyscopeEnvironment shutdown ", Timer::now());
}
void Manager::addSurface(PolySurface& psurf) {
  std::vector<std::vector<size_t>> faces;
  for (auto& face : psurf.allFaces()) {
    faces.push_back(psurf.verticesAroundFace(face));
  }
  auto digsurf = polyscope::registerSurfaceMesh(defaultRegisteredMeshName,
                                                psurf.positions(), faces);
  digsurf->setSurfaceColor(defaultMeshColor);
  digsurf->setTransparency(defaultMeshTransparency)->setEnabled(true);
  // updateSelection();
}
void Manager::storeSelectedAssociatedFacesInMap() {
  auto voxelId = selectedElementId;
  if (voxelId < 0 || static_cast<size_t>(voxelId) >= nas.voxelList.size()) {
    promptText = "ERROR: Current voxel " + std::to_string(selectedElementId) +
                 " in storeSelectedAssociatedFacesInList() is not found";
    log->add(LogLevel::ERROR,
             "ERROR: Current voxel " + std::to_string(selectedElementId) +
                 " in storeSelectedAssociatedFacesInList() is not found ",
             Timer::now());
    return;
  }
  for (auto faceId : nas.voxelList[voxelId].associatedFaceIds) {
    selectedAssociatedFacesMap[faceId].push_back(voxelId);
  }
}
void Manager::paintFacesOn(
    std::string& meshName, std::string& quantityName,
    std::unordered_map<size_t, std::vector<DGtalUint>>& faceMap) {
  // find all associated faces[DGtal face Id] on polyscope mesh
  auto digsurf = polyscope::getSurfaceMesh(meshName);
  if (digsurf == nullptr) {
    return;  // It never happens because the polyscope will fail without return a nullptr.
  }
  // TODO: make function flexible to use different dynamic color map
  currentfacesColor.clear();
  currentfacesColor.resize(digsurf->nFaces(), defaultMeshColor);
  auto& usedColorMap = polyscope::render::engine->getColorMap("turbo");
  auto tempMinMax = AccumulationSpace::getMinMaxVotesCountFrom(nas.voxelList);
  // Retrive the color value from the color map
  log->add(LogLevel::DEBUG,
           "In " + quantityName + " Map size: ", faceMap.size());

  // Second loop indicate that not only one voxel is mapped to a face (not vice versa)
  for (auto f : faceMap) {
    for (auto vId : f.second) {
      double tempValue = (nas.voxelList[vId].votes * 1.0l - tempMinMax.first) /
                         (tempMinMax.second - tempMinMax.first);
      auto tempColor = usedColorMap.getValue(tempValue);
      for (auto fId : nas.voxelList[vId].associatedFaceIds) {
        currentfacesColor[f.first] = tempColor;
      }
    }
  }

  if (quantityName == defaultMeshColorQuantityName) {
    defaultfacesColor = currentfacesColor;
  }
  digsurf->addFaceColorQuantity(quantityName, currentfacesColor)
      ->setEnabled(true);
}
void Manager::findLoadedAssociatedAccumulationsByFaceId() {
  auto faceId = selectedElementId;
  auto it = globalFaceMap.find(faceId);
  if (faceId < 0 || it == globalFaceMap.end()) {
    promptText =
        "ERROR: Current face " + std::to_string(selectedElementId) +
        " is not found in findLoadedAssociatedAccumulationsByFaceId() ";
    log->add(
        LogLevel::DEBUG,
        "Current face " + std::to_string(selectedElementId) +
            " is not found in findLoadedAssociatedAccumulationsByFaceId() ",
        Timer::now());
    return;
  }
  for (auto l : it->second) {
    associatedAccumulationIds.insert(l);  // No duplicates
  }
}
void Manager::buttonResetSelectedColorFacesEvent() {
  auto tempMesh = polyscope::getSurfaceMesh(defaultRegisteredMeshName);
  if (tempMesh != nullptr) {
    tempMesh->setEnabled(false);
  }
  currentfacesColor.clear();
  currentfacesColor.resize(tempMesh->nFaces(), tempMesh->getSurfaceColor());
  selectedAssociatedFacesMap.clear();
  tempMesh->addFaceColorQuantity(associatedMeshColorQuantityName,
                                 currentfacesColor);
  tempMesh->setEnabled(true);
  promptText = "[" + tempMesh->typeName() + "] Reset faces color";
}
void Manager::buttonResetSelectedColorVoxelsEvent() {
  auto tempVoxels = polyscope::getPointCloud("Selected Associated Voxels");
  if (tempVoxels == nullptr) {
    return;
  }
  selectedAssociatedPoints.clear();
  associatedAccumulationIds.clear();
  tempVoxels = polyscope::registerPointCloud("Selected Associated Voxels",
                                             selectedAssociatedPoints);
  promptText = "[" + tempVoxels->typeName() + "] Reset voxels color";
}
void Manager::setImguiCustomPanel() {
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
  ImGui::Separator();

  ImGui::Text("Selection settings");

  if (ImGui::Button("Reset selected color faces")) {
    buttonResetSelectedColorFacesEvent();
  }

  if (ImGui::Button("Reset selected color voxels")) {
    buttonResetSelectedColorVoxelsEvent();
  }

  if (ImGui::Button(accBtnPressed2 ? "Show default color map"
                                   : "Hide default color map")) {
    if (accBtnPressed2) {
      polyscope::getSurfaceMesh(defaultRegisteredMeshName)
          ->addFaceColorQuantity(defaultMeshColorQuantityName,
                                 defaultfacesColor);
    } else {
      polyscope::getSurfaceMesh(defaultRegisteredMeshName)
          ->removeQuantity(defaultMeshColorQuantityName);
      // polyscope::removePointCloud("Cluster Voxels");
    }
    accBtnPressed2 = !accBtnPressed2;
  }
  ImGui::SameLine();
  // thrid line
  if (ImGui::Button(accBtnPressed3 ? "Hide incident ray"
                                   : "Show incident ray")) {
    // check face selected
    // currentCachedPointCloud(accumulation voxel) updace
    if (accBtnPressed3) {
      // TODO: implement the incident ray
    } else {
    }
    accBtnPressed3 = !accBtnPressed3;
  }
  ImGui::Text("Debugging Info:");
  ImGui::Text("%s", promptText.c_str());
  ImGui::Separator();

  ImGui::Text("Neighborhood clustering algorithm");
  if (ImGui::Button(accBtnPressed4 ? "Show Neighbour clusters"
                                   : "Hide Neighbour clusters")) {
    if (accBtnPressed4) {
      AccumulationAlgorithms::NeighbourClusterAlgo nca;
      nca.clearCluster();
      nca.buildCluster(nas.voxelList, 0, 0.0, log, imguiAlgoStep);
      paintCluster(nca);
    } else {
      polyscope::getPointCloud("Cluster Voxels")->setEnabled(false);
      polyscope::getSurfaceMesh(defaultRegisteredMeshName)
          ->removeQuantity("Cluster faces color");
    }
    accBtnPressed4 = !accBtnPressed4;
  }
  if (ImGui::SliderInt("Traverse step", &imguiAlgoStep, 1,
                       defaultAlgoMaxStep)) {
    // get current defined cluster
    AccumulationAlgorithms::NeighbourClusterAlgo nca;
    nca.clearCluster();
    nca.buildCluster(nas.voxelList, 0, 0.0, log, imguiAlgoStep);
    promptText = "Traverse step: " + std::to_string(imguiAlgoStep);
    mouseDragSliderEvent(imguiAlgoStep, nca);
  }

  ImGui::Separator();
  ImGui::Text("Radius clustering algorithm");
  if (ImGui::Button(accBtnPressed5 ? "Show static radius clusters"
                                   : "Hide radius clusters")) {
    if (accBtnPressed5) {
      AccumulationAlgorithms::RadiusClusterAlgo rca;
      rca.clearCluster();
      rca.buildCluster(nas.voxelList, 0, 0.0, log, currentPolysurf, 0);
      paintCluster(rca);
    } else {
      polyscope::getPointCloud("Cluster Voxels")->setEnabled(false);
      polyscope::getSurfaceMesh(defaultRegisteredMeshName)
          ->removeQuantity("Cluster faces color");
    }
    accBtnPressed5 = !accBtnPressed5;
  }
  if (ImGui::Button(accBtnPressed6 ? "Show dynamic radius clusters"
                                   : "Hide radius clusters")) {
    if (accBtnPressed6) {
      AccumulationAlgorithms::RadiusClusterAlgo rca;
      rca.clearCluster();
      rca.buildCluster(nas.voxelList, 0, 0.0, log, currentPolysurf, 1);
      paintCluster(rca);
    } else {
      polyscope::getPointCloud("Cluster Voxels")->setEnabled(false);
      polyscope::getSurfaceMesh(defaultRegisteredMeshName)
          ->removeQuantity("Cluster faces color");
    }
    accBtnPressed6 = !accBtnPressed6;
  }
  if (ImGui::SliderInt("Radius length", &imguiAlgoStep, 1,
                       defaultAlgoMaxStep)) {
    // TODO
  }

  ImGui::Separator();
}
// It's interesting to see how many times this functioanlity is called
size_t Manager::convertMeshElementIdInPolyscope(size_t elementId) {
  if (elementId < 0) {
    promptText = "ERROR: In convertMeshElementIdInPolyscope() element < 0";
    log->add(LogLevel::ERROR,
             "ERROR: In convertMeshElementIdInPolyscope() element < 0 ",
             Timer::now());
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
void Manager::paintSelectedAssociatedAccumulations() {
  // find all associated voxels[polyscope vector Id] in polyscope point cloud
  auto primaryPointCloud = polyscope::getPointCloud("Primary Voxels");
  if (primaryPointCloud == nullptr) {
    return;
  }
  primaryPointCloud->setEnabled(false);

  selectedAssociatedPoints.clear();
  std::vector<double> vScalar;
  for (auto accId : associatedAccumulationIds) {
    selectedAssociatedPoints.push_back(nas.pointList[accId]);
    vScalar.push_back(accmulationScalarValues[accId]);
    log->add(LogLevel::DEBUG, accId, " face and accumulation is ",
             accmulationScalarValues[accId]);
  }

  log->add(LogLevel::DEBUG, associatedAccumulationIds.size(),
           " voxels selected ", Timer::now());
  // paint selected voxels
  polyscope::PointCloud* tempPointCloud = polyscope::registerPointCloud(
      "Selected Associated Voxels", selectedAssociatedPoints);
  tempPointCloud->setPointRadius(defaultPointRadius);
  defaultColorMapRange =
      AccumulationSpace::getMinMaxVotesCountFrom(nas.voxelList);
  auto q1 = tempPointCloud->addScalarQuantity("AssoAcc", vScalar);
  q1->setColorMap(defaultColorMap);
  q1->setMapRange(defaultColorMapRange);
  q1->setEnabled(true);

  tempPointCloud->setTransparency(defaultPointTransparency);
  tempPointCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
  tempPointCloud->setEnabled(true);
}
void Manager::paintColoredClusterPointsIn(
    std::string pointCloudName, AccumulationAlgorithms::ClusterAlgoBase& base) {
  // cluster position map with scalar
  std::vector<double> vScalar;
  base.pointList.clear();
  for (int i = 1; i < base.cluster.size(); i++) {
    for (const auto posHashValue : base.cluster[i]) {
      auto tempV = base.voxelMap.at(posHashValue);
      auto tempPos =
          glm::vec3(tempV.position[0], tempV.position[1], tempV.position[2]);
      base.pointList.push_back(tempPos);
      vScalar.push_back(i);
    }
  }
  // coloring cluster voxels
  polyscope::PointCloud* tempPointCloud =
      polyscope::registerPointCloud(pointCloudName, base.pointList);
  tempPointCloud->setPointRadius(defaultPointRadius);
  tempPointCloud->setEnabled(true);
  // TODO: get selected color map and map range
  defaultColorMapRange = std::make_pair(0, base.clusterLabel);
  auto quantity =
      tempPointCloud->addScalarQuantity("Cluster points color", vScalar);
  quantity->setColorMap(defaultColorMap);
  quantity->setMapRange(defaultColorMapRange);
  quantity->setEnabled(true);
  log->add(LogLevel::INFO, "Cluster voxels painted ", Timer::now());
}
void Manager::paintColoredClusterFacesOn(
    std::string meshName, std::string quantityName,
    AccumulationAlgorithms::ClusterAlgoBase& base) {
  auto digsurf = polyscope::getSurfaceMesh(meshName);
  if (digsurf == nullptr) {
    return;
  }
  currentfacesColor.clear();
  currentfacesColor.resize(digsurf->nFaces(), defaultMeshColor);
  auto& usedColorMap = polyscope::render::engine->getColorMap(defaultColorMap);

  // TODO: get selected color map and map range
  for (int i = 1; i < base.cluster.size(); i++) {
    double tempValue =
        (i * 1.0l - defaultColorMapRange.first) /
        (defaultColorMapRange.second - defaultColorMapRange.first);
    auto tempColor = usedColorMap.getValue(tempValue);
    for (const auto posHashValue : base.cluster[i]) {
      for (auto fId : base.voxelMap.at(posHashValue).associatedFaceIds) {
        currentfacesColor[fId] = tempColor;
      }
    }
  }
  digsurf->addFaceColorQuantity(quantityName, currentfacesColor)
      ->setEnabled(true);
  log->add(LogLevel::INFO, "Cluster faces color painted ", Timer::now());
}
void Manager::paintCluster(AccumulationAlgorithms::ClusterAlgoBase& base) {
  auto primaryPointCloud = polyscope::getPointCloud("Primary Voxels");
  if (primaryPointCloud == nullptr) {
    return;
  }
  primaryPointCloud->setEnabled(false);

  // coloring cluster voxels
  paintColoredClusterPointsIn("Cluster Voxels", base);
  // coloring cluster faces
  paintColoredClusterFacesOn(defaultRegisteredMeshName, "Cluster faces color",
                             base);
}

void Manager::mouseSelectStructureEvent(ImGuiIO& io) {
  // polyscope default left-click selection
  if (io.MouseDoubleClicked[0]) {
    if (polyscope::pick::haveSelection()) {
      std::pair<polyscope::Structure*, size_t> selection =
          polyscope::pick::getSelection();

      if (selection.second < 0) {
        promptText = "ERROR: In mouseSelectAccumulation() selection.second < 0";
        log->add(LogLevel::ERROR,
                 "ERROR: In mouseSelectAccumulation() selection.second < 0 ",
                 Timer::now());
        return;
      }
      if (selection.first->typeName() == "Point Cloud") {
        clickCount++;
        selectedElementId = selection.second;
        storeSelectedAssociatedFacesInMap();
        // paintSelectedAssociatedFaces();
        paintFacesOn(defaultRegisteredMeshName, associatedMeshColorQuantityName,
                     selectedAssociatedFacesMap);
      }
      if (selection.first->typeName() == "Surface Mesh") {
        clickCount++;
        selectedElementId = convertMeshElementIdInPolyscope(selection.second);
        findLoadedAssociatedAccumulationsByFaceId();
        paintSelectedAssociatedAccumulations();  // one voxel for now
                                                 // vector<accId>
      }
    }
  }
}
void Manager::mouseDragSliderEvent(
    int& step, AccumulationAlgorithms::ClusterAlgoBase& base) {
  if (step < 0) {
    step = 0;
  }
  if (step > defaultAlgoMaxStep) {
    step = defaultAlgoMaxStep;
  }
  paintCluster(base);
  log->add(LogLevel::INFO, "Traverse step: ", step, " painted ", Timer::now());
}
void Manager::mouseEventCallback(ImGuiIO& io) {
  mouseSelectStructureEvent(io);
}
void Manager::setImguiBegin() {
  srand((unsigned)time(NULL));
  ImGui::Begin("Editing tools");
}
void Manager::setImguiEnd() {
  ImGui::End();
}
void Manager::setImguiIO(ImGuiIO& io) {
  io.FontGlobalScale = defaultImguiDPIRatio;
}

void Manager::callback() {
  setImguiBegin();
  setImguiCustomPanel();
  // io
  ImGuiIO& io = ImGui::GetIO();
  setImguiIO(io);
  mouseEventCallback(io);
  setImguiEnd();
  // updateSelection();}
}
void Manager::buildHashMap2Voxels() {
  for (auto voxel : nas.voxelList) {
    globalHashMap[AccumulationSpace::accumulationHash(voxel.position)] = voxel;
  }
}
void Manager::buildFaceMap2Voxels() {
  for (size_t i = 0; i < nas.voxelList.size(); i++) {
    const auto& voxel = nas.voxelList[i];
    for (auto faceId : voxel.associatedFaceIds) {
      globalFaceMap[faceId].push_back(i);
    }
  }
}

}  // namespace PolyscopeEnvironment