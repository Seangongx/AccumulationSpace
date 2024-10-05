#include "PolyscopeEnvironment.h"

namespace PolyscopeEnvironment {


Manager::Manager(const std::string& meshFile, const std::string& accFile) {
  // Initialize polyscope
  polyscope::options::programName = "PolyAccEdit - (DGtalToolsContrib) " + Timer::now();
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
  nas.buildFromFile(accFile);
  addPointCloud("Primary Voxels", nas.pointList, 0.008, glm::vec3(1.0f, 1.0f, 1.0f));

  promptText = "Select nothing at the beginning\n";
  // build necessary data structure
  buildHashMap2Voxels();
  buildFaceMap2Voxels();

  polyscope::state::userCallback = [this]() { this->callback(); };
  polyscope::show();
}


// Visualize accumulation set in space
void Manager::addPointCloud(std::string structName, PointLists structPoints, double structRadius, PsColor structColor) {

  polyscope::PointCloud* psCloud = polyscope::registerPointCloud(structName, structPoints);
  psCloud->setPointRadius(structRadius);
  // set accmuluation votes as scalar quantity)
  accmulationScalarValues.resize(nas.voxelList.size());
  for (size_t i = 0; i < structPoints.size(); i++) {
    accmulationScalarValues[i] = nas.voxelList[i].votes;
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
    // auto q2 = psCloud->addScalarQuantity("accumulationTransparency",
    // accumulationTransparency); psCloud->setTransparencyQuantity(q2);

    // polyscope::loadColorMap("sampleColorMap",
    // "/home/adam/Desktop/AccumulationSpace/samples/sample_colormap.png");
  }
  // structureTypes["InputPoints"] = POINTCLOUD;
}

void Manager::addSurface(PolySurface& psurf) {
  std::vector<std::vector<size_t>> faces;
  for (auto& face : psurf.allFaces()) {
    faces.push_back(psurf.verticesAroundFace(face));
  }
  auto digsurf = polyscope::registerSurfaceMesh("InputMesh", psurf.positions(), faces);
  digsurf->setSurfaceColor(defaultMeshColor);
  digsurf->setTransparency(defaultMeshTransparency)->setEnabled(true);
  // updateSelection();
}


void Manager::storeAllAssociatedFacesInList() {
  auto voxelId = selectedElementId;
  if (voxelId < 0 || static_cast<size_t>(voxelId) >= nas.voxelList.size()) {
    return;
  }
  selectedAssociatedFacesMap[selectedElementId] = nas.voxelList[voxelId].associatedFaceIds;
}

void Manager::paintAllAssociatedFaces() {
  // find all associated faces[DGtal face Id] on polyscope mesh
  auto digsurf = polyscope::getSurfaceMesh("InputMesh");
  if (digsurf == nullptr) {
    return;
  }
  // TODO: make function flexible to use different color map
  currentfacesColor.resize(digsurf->nFaces(), digsurf->getSurfaceColor());
  auto& usedColorMap = polyscope::render::engine->getColorMap("turbo");
  auto tempMinMax = AccumulationSpace::getMinMaxVotesCountFrom(nas.voxelList);
  double tempValue = (nas.voxelList[selectedElementId].votes * 1.0l - tempMinMax.first) / tempMinMax.second;
  auto tempColor = usedColorMap.getValue(tempValue);

  for (auto faceId : selectedAssociatedFacesMap[selectedElementId]) {
    currentfacesColor[faceId] = tempColor;
  }
  digsurf->addFaceColorQuantity(defaultMeshColorQuantityName, currentfacesColor)->setEnabled(true);
}

void Manager::findLoadedAssociatedAccumulationsByFaceId() {
  auto faceId = selectedElementId;
  auto it = globalFaceMap.find(faceId);
  if (faceId < 0 || it == globalFaceMap.end()) {
    promptText = "ERROR: Current face " + std::to_string(selectedElementId) +
                 " in findLoadedAssociatedAccumulationsByFaceId() faceId < 0";
    return;
  }
  for (auto l : it->second) {
    associatedAccumulationIds.insert(l);
  }
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
    tempMesh->addFaceColorQuantity(defaultMeshColorQuantityName, currentfacesColor);
    tempMesh->setEnabled(true);
    promptText = "[" + tempMesh->typeName() + "] Reset faces color";
  }

  if (ImGui::Button("Reset selection color voxels")) {
    auto tempVoxels = polyscope::getPointCloud("Selected Associated Voxels");
    if (tempVoxels == nullptr) {
      return;
    }
    selectedAssociatedPoints.clear();
    associatedAccumulationIds.clear();
    tempVoxels = polyscope::registerPointCloud("Selected Associated Voxels", selectedAssociatedPoints);
    promptText = "[" + tempVoxels->typeName() + "] Reset voxels color";
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


// It's interesting to see how many times this functioanlity is called
size_t Manager::convertMeshElementIdInPolyscope(size_t elementId) {
  if (elementId < 0) {
    promptText = "ERROR: In convertMeshElementIdInPolyscope() element < 0";
    std::cerr << "ERROR: In convertMeshElementIdInPolyscope() element < 0" << std::endl;
    return static_cast<size_t>(-1);
  }
  auto facePickIndStart = currentPolysurf.nbVertices();
  auto edgePickIndStart = facePickIndStart + currentPolysurf.nbFaces();
  if (elementId < facePickIndStart) {        // vertex
  } else if (elementId < edgePickIndStart) { // face
    elementId -= facePickIndStart;
  } else { // unknown
    return static_cast<size_t>(-1);
  }
  return elementId;
}

void Manager::mouseSelectIndexTest(ImGuiIO& io) {

  std::pair<polyscope::Structure*, size_t> selection = polyscope::pick::getSelection();

  if (polyscope::pick::haveSelection()) {
    size_t nb = 0;
    // face selected
    if (selection.first == nullptr) {
      clickCount++;
      promptText = "[" + std::to_string(clickCount) + "] nothing selected";
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
    std::cout << accId << " face and accumulation is " << accmulationScalarValues[accId] << std::endl;
  }

  std::cout << associatedAccumulationIds.size() << std::endl;
  // paint selected voxels
  polyscope::PointCloud* tempPointCloud =
      polyscope::registerPointCloud("Selected Associated Voxels", selectedAssociatedPoints);
  tempPointCloud->setPointRadius(0.008);
  turboColorMapRange = AccumulationSpace::getMinMaxVotesCountFrom(nas.voxelList);
  auto q1 = tempPointCloud->addScalarQuantity("AssoAcc", vScalar);
  q1->setColorMap("turbo");
  q1->setMapRange(turboColorMapRange);
  q1->setEnabled(true);

  tempPointCloud->setTransparency(0.8);
  tempPointCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
  tempPointCloud->setEnabled(true);
}

void Manager::mouseSelectAccumulation(ImGuiIO& io) {
  // polyscope default left-click selection
  if (polyscope::pick::haveSelection()) {
    std::pair<polyscope::Structure*, size_t> selection = polyscope::pick::getSelection();
    // if double clicked an accumulation voxel
    if (io.MouseDoubleClicked[0]) {
      if (selection.second < 0) {
        promptText = "ERROR: In mouseSelectAccumulation() selection.second < 0";
        std::cerr << "ERROR: In mouseSelectAccumulation() selection.second < 0" << std::endl;
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
          std::cerr << "ERROR: Find a " << associatedAccumulationIds.size() << " map" << std::endl;
          return;
        }
        paintSelectedAssociatedAccumulations(); // one voxel for now
                                                // vector<accId>
      }
      promptText = "[Double click " + selection.first->typeName() + "]: " + std::to_string(selectedElementId);
    }
  }
}

void Manager::mouseEventCallback(ImGuiIO& io) {

  // mouseSelectFaces(io)
  // mouseSelectIndexTest(io);
  mouseSelectAccumulation(io);
}


void Manager::setImguiBegin() {
  srand((unsigned)time(NULL));
  ImGui::Begin("Editing tools");
}

void Manager::setImguiEnd() {
  ImGui::Text("%s", promptText.c_str());
  ImGui::End();
}

void Manager::setImguiIO(ImGuiIO& io) {
  float dpiScale = 1.8f; // DPI ratio
  io.FontGlobalScale = dpiScale;
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
  nas.acclog.add(LogLevel::INFO, "HashMap Finished size: ", globalHashMap.size());
}

void Manager::buildFaceMap2Voxels() {
  for (size_t i = 0; i < nas.voxelList.size(); i++) {
    const auto& voxel = nas.voxelList[i];
    for (auto faceId : voxel.associatedFaceIds) {
      globalFaceMap[faceId].push_back(i);
    }
  }
  nas.acclog.add(LogLevel::INFO, "Face Finished size: ", globalFaceMap.size());
}

} // namespace PolyscopeEnvironment