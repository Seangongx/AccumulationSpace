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
 * @file segmentFromAcc.cpp
 * @ingroup geometry3d
 * @author Bertrand Kerautret (\c bertrand.kerautret@univ-lyon2.fr )
 * LIRIS (CNRS, UMR 5205), University of Lyon 2, France
 * @author Xun Gong (\c sean.gong814@gmail.com )
 *
 * @date 2023/10/06
 *
 * Source file of the tool segmentFromAcc
 *
 * This file is part of the DGtal library/DGtalTools-contrib Project.
 */

///////////////////////////////////////////////////////////////////////////////
#include <chrono> // for timekeeping
#include <cstddef>
#include <ctime> // for time display
#include <fstream>
#include <iostream>
// DGtal library
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/ImageContainerBySTLMap.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/RandomColorMap.h"
#include "DGtal/io/readers/MeshReader.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/io/writers/MeshWriter.h"
// Other dependencies
#include "AccVoxel.h"
#include "AccVoxelHelper.h"
#include "CLI11.hpp"
#include "Menu.h"
#include "Timer.h"

///////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace DGtal;
// #define DEBUG
///////////////////////////////////////////////////////////////////////////////

/**
 @page segmentFromAcc

 @brief get accumulation or confidence clusters based on the accumulation image meanwhile
  paint the associate colored segement on the original mesh.

 @code
 Typical use example:
     [path to the project] segmentFromAcc -i file.obj -o file.off -m modevalue -s shadervalue -t thetavalue

 Usage: ./expe/segmentFromAcc [OPTIONS] 1 [2]

 Positionals:
   1 TEXT:FILE REQUIRED                  an input mesh file in .obj or .off format.
   2 TEXT:FILE REQUIRED                  n output file
   3 VALUE:REQUIRED                      select confidence (2), accumulation (1), and both by default(0) mode
   4 VALUE:REQUIRED                      select HueShader (1) or GradientShader (0) mode
   5 VALUE:REQUIRED                      select threshold value for confidence mode

 Options:
   -h,--help                             Print this help message and exit
   -i,--input TEXT:FILE REQUIRED         an input mesh file in .obj or .off format.
   -o,--output TEXT                      an output file

 @endcode

 @b Example:

 @code
 segmentFromAcc $DGtal/examples/samples/am_beech2-3.off am_beech2-3Acc.dat 2 0 0.8
 @endcode

 @see
 @ref segmentFromAcc.cpp

 */

#pragma region Type Definitions

// define message function
enum LogLevel { INFO, WARNING, ERROR, DEBUG };

void printMessage(const std::string& message, LogLevel level = INFO) {
  const char* levelStr;
  switch (level) {
  case INFO:
    levelStr = "[INFO]";
    break;
  case WARNING:
    levelStr = "[WARNING]";
    break;
  case ERROR:
    levelStr = "[ERROR]";
    break;
  case DEBUG:
    levelStr = "[DEBUG]";
    break;
  }

  // 获取当前时间
  std::time_t now = std::time(nullptr);
  char buf[100];
  std::strftime(buf, sizeof(buf), "%d %H:%M:%S", std::localtime(&now));
  std::cout << buf << " " << levelStr << " " << message << std::endl;
}

// DGtal definitions
typedef DGtal::SpaceND<3> Space;
typedef DGtal::HyperRectDomain<Space> Dom;
typedef DGtal::uint32_t Uint;
typedef DGtal::Z3i::Point Point3D; // Interger 3D point ( Z3i )
typedef DGtal::PointVector<1, DGtal::int32_t> Point1D;
typedef DGtal::Mesh<DGtal::Z3i::RealPoint> RealMesh;
// STD definiations
typedef map<Uint, AccVoxel> HashMapVoxel; // id mapping to voxels
typedef priority_queue<int, vector<int>, CompareConfsAsc> Test;

#pragma endregion

template <typename MapType, typename KeyType>
bool mapKeychecker(const MapType& map, const KeyType& key) {
  auto it = map.find(key);
  if (it != map.end()) return true;
  return false;
}

bool isOnBoundary(const Point3D& p, std::pair<Point3D, Point3D> bbox) {
  for (int i = 0; i < 3; i++) {
    if (p[i] == bbox.first[i] || p[i] == bbox.second[i]) return true;
  }
  return false;
}

/// @brief Push surrounding voxels which are unvisited and no label in local queue
/// @param voxel
/// @param mapVoxel
/// @param queue
void markNeighbours(const AccVoxel& voxel, HashMapVoxel& mapVoxel, std::queue<AccVoxel>& queue) {
#ifdef DEBUG
  cout << "DEBUG: Current center is " << voxel.label << " -> " << voxel.p << endl;
#endif
  int gridStep = 1;
  std::vector<std::tuple<int, int, int>> neighbors = {{-gridStep, -gridStep, -gridStep},
                                                      {-gridStep, -gridStep, 0},
                                                      {-gridStep, -gridStep, gridStep},
                                                      {-gridStep, 0, -gridStep},
                                                      {-gridStep, 0, 0},
                                                      {-gridStep, 0, gridStep},
                                                      {-gridStep, gridStep, -gridStep},
                                                      {-gridStep, gridStep, 0},
                                                      {-gridStep, gridStep, gridStep},
                                                      {0, -gridStep, -gridStep},
                                                      {0, -gridStep, 0},
                                                      {0, -gridStep, gridStep},
                                                      {0, 0, -gridStep},
                                                      {0, 0, gridStep},
                                                      {0, gridStep, -gridStep},
                                                      {0, gridStep, 0},
                                                      {0, gridStep, gridStep},
                                                      {gridStep, -gridStep, -gridStep},
                                                      {gridStep, -gridStep, 0},
                                                      {gridStep, -gridStep, gridStep},
                                                      {gridStep, 0, -gridStep},
                                                      {gridStep, 0, 0},
                                                      {gridStep, 0, gridStep},
                                                      {gridStep, gridStep, -gridStep},
                                                      {gridStep, gridStep, 0},
                                                      {gridStep, gridStep, gridStep}};

  for (const auto& [dx, dy, dz] : neighbors) {
    // locate the adjacent voxel id
    Point3D pTemp(voxel.p[0] + dx, voxel.p[1] + dy, voxel.p[2] + dz);
    auto idTemp = AccVoxelHelper::accumulationHash(pTemp);

    if (mapKeychecker(mapVoxel, idTemp)) {
      const auto& voxelTemp = mapVoxel.at(idTemp);
      // if (isOnBoundary(pAdjacent, bbox)) // skip the boundary voxel
      //   continue;
      if (!voxelTemp.visited && pTemp != voxel.p && voxelTemp.label == 0) // skip the visited voxel and the center
      {
        mapVoxel.at(idTemp).label = voxel.label; // update the current cluster label in global map
        queue.push(mapVoxel.at(idTemp));

#ifdef DEBUG
        cout << "DEBUG: Label in [" << voxel.label << "] and push " << mapVoxel.at(idTemp).p << endl;
#endif
      }
    }
  }
}

/// @brief Using a breadth-first traversal strategy to label all clusters
/// @tparam TypePQ
/// @param mapVoxel
/// @param globalPQ
/// @param clusterLabel
template <typename TypePQ>
void processAccLabel(HashMapVoxel& mapVoxel, TypePQ& globalPQ, Uint& clusterLabel,
                     std::vector<std::vector<Uint>>& cluster) {
  std::queue<AccVoxel> localQ;            // maintain the local visited:
  cluster.push_back(std::vector<Uint>()); // init the empty cluster 0
  Uint countPush = 0;

  for (auto v : mapVoxel) globalPQ.push(v.second);

  printMessage("Loaded global queue size is " + std::to_string(globalPQ.size()));

  // LOOP I: select the next voxel with the highest votes
  while (!globalPQ.empty()) {
    auto vCurrent = globalPQ.top();
    auto pCurrent = vCurrent.p;
    auto idCurrent = AccVoxelHelper::accumulationHash(vCurrent.p);

    if (mapVoxel.at(idCurrent).visited) {
      globalPQ.pop();
#ifdef DEBUG
      printMessage("Pop(G): " + std::to_string(idCurrent) + " and remain " + std::to_string(globalPQ.size()));
#endif
      continue;
    }

    // LOOP II: traverse the adjacent voxels

    // Form a new cluster, push the first voxel and update the current cluster label
    mapVoxel.at(idCurrent).visited = true;
    mapVoxel.at(idCurrent).label = ++clusterLabel;
    vCurrent.label = clusterLabel;
    cluster.push_back(std::vector<Uint>());
    cluster[clusterLabel].push_back(idCurrent);

#ifdef DEBUG
    cout << "DEBUG: (" << ++countPush << ") push " << mapVoxel.at(idCurrent).p << " in cluster " << clusterLabel
         << " and set visited" << endl;
    cout << "DEBUG: Current cluster center is " << pCurrent << " -> " << vCurrent.label << endl;

    cout << "DEBUG: Remain(G): " << globalPQ.size() << endl;
#endif

    markNeighbours(vCurrent, mapVoxel, localQ);
    while (!localQ.empty()) {
      auto vAdjacent = localQ.front();
      auto pAdjacent = vAdjacent.p;
      auto idAdjacent = AccVoxelHelper::accumulationHash(vAdjacent.p);

#ifdef DEBUG
      cout << "DEBUG: Visiting [" << clusterLabel << "] : " << pAdjacent << endl;
      cout << "DEBUG: Remain(L): " << localQ.size() << endl;
#endif

      if (mapVoxel.at(idAdjacent).visited) {
        localQ.pop();
        continue;
      }
      if (mapVoxel.at(idAdjacent).label == 0) {
        cout << "ERROR: " << pAdjacent << " is not visited and no label" << endl;
      }

      // mark visited, store in a cluster and label rest neighbors
      mapVoxel.at(idAdjacent).visited = true;
      cluster[clusterLabel].push_back(idAdjacent);
      localQ.pop();
      markNeighbours(vAdjacent, mapVoxel, localQ);
    }
  }

  cout << "SUCCESS: Found " << clusterLabel << " clusters" << endl;
}

/// @brief Overload the processLabel function with a threshold
/// @tparam TypePQ
/// @param mapVoxel
/// @param globalPQ
/// @param clusterLabel
/// @param theta
template <typename TypePQ>
void processConfLabel(HashMapVoxel& mapVoxel, TypePQ& globalPQ, Uint& clusterLabel,
                      std::vector<std::vector<Uint>>& cluster, float theta = 0.0f) {
  Uint countFilterTimes = 0;
  Uint sizeLocalQueue = 0;
  std::queue<AccVoxel> localQ;            // maintain the local visited:
  cluster.push_back(std::vector<Uint>()); // init the empty cluster 0
  Uint countPush = 0;

  for (auto& v : mapVoxel) {
    if (v.second.confs > theta && v.second.votes > 0) {
      globalPQ.push(v.second);
    } else {
      // label 0 was been igonred
      countFilterTimes++;
      v.second.visited = true;
    }
  }
  cout << "SUCCESS: Loaded global queue size is " << globalPQ.size() << endl;
  sizeLocalQueue = globalPQ.size();

  // LOOP I: select the next voxel with the highest votes
  while (!globalPQ.empty()) {
    auto vCurrent = globalPQ.top();
    auto pCurrent = vCurrent.p;
    auto idCurrent = AccVoxelHelper::accumulationHash(vCurrent.p);

    if (mapVoxel.at(idCurrent).visited) {
      globalPQ.pop();
#ifdef DEBUG
      cout << "DEBUG: Pop(G): " << pCurrent << " and remain " << globalPQ.size() << endl;
#endif
      continue;
    }

    // LOOP II: traverse the adjacent voxels

    // Select a new cluster starting voxel update the current cluster label
    mapVoxel.at(idCurrent).visited = true;
    mapVoxel.at(idCurrent).label = ++clusterLabel;
    vCurrent.label = clusterLabel;
    cluster.push_back(std::vector<Uint>());
    cluster[clusterLabel].push_back(idCurrent);

#ifdef DEBUG
    cout << "DEBUG: (" << ++countPush << ") push " << mapVoxel.at(idCurrent).p << " in cluster " << clusterLabel
         << " and set visited" << endl;
    cout << "DEBUG: Current cluster center is " << pCurrent << " -> " << vCurrent.label << endl;
    cout << "DEBUG: Remain(G): " << globalPQ.size() << endl;
#endif

    markNeighbours(vCurrent, mapVoxel, localQ);
    while (!localQ.empty()) {
      auto vAdjacent = localQ.front();
      auto pAdjacent = vAdjacent.p;
      auto idAdjacent = AccVoxelHelper::accumulationHash(vAdjacent.p);

#ifdef DEBUG
      cout << "DEBUG: Visiting [" << clusterLabel << "] : " << pAdjacent << endl;
      cout << "DEBUG: Remain(L): " << localQ.size() << endl;
#endif

      if (mapVoxel.at(idAdjacent).visited) {
        localQ.pop();
        continue;
      }
      if (mapVoxel.at(idAdjacent).label == 0) {
        cout << "ERROR: " << pAdjacent << " is not visited and no label" << endl;
      }

      // mark visited, store in a cluster and label rest neighbors
      mapVoxel.at(idAdjacent).visited = true;
      cluster[clusterLabel].push_back(idAdjacent);

      localQ.pop();

      markNeighbours(vAdjacent, mapVoxel, localQ);
    }
  }

  cout << "SUCCESS: Found " << clusterLabel << " clusters and " << sizeLocalQueue << " voxels" << endl;
  cout << "FILTER: Filtered " << countFilterTimes << " voxels" << endl;
}

void computeConfidence(HashMapVoxel& mapVoxel, float theta) {
  for (auto& [key, voxel] : mapVoxel) voxel.confs /= voxel.votes;
}

template <typename Shader>
void shadeFaces(ofstream& fs, HashMapVoxel& mapVoxel, RealMesh& aMesh, Uint clusterLabel) {
  Shader colorMap(0, clusterLabel);
  colorMap.addColor(Color::Red);
  colorMap.addColor(Color::Yellow);
  colorMap.addColor(Color::Blue);

  for (const auto& [key, voxel] : mapVoxel) {
    if (voxel.label == 0) {
      cout << "ERROR: " << voxel.p << " is " << voxel.visited << " and no label" << endl;
      continue;
    }
    Color cTemp = colorMap(voxel.label);

    // SDP color
    fs << voxel.p[0] << " " << voxel.p[1] << " " << voxel.p[2] << " " << int(cTemp.red()) << " " << int(cTemp.green())
       << " " << int(cTemp.blue()) << " " << endl;
    // Faces color
    for (auto f : voxel.faces) {
      aMesh.setFaceColor(f, cTemp);
    }
  }
}

void outputGradientShader(ofstream& fs, HashMapVoxel& mapVoxel, RealMesh& aMesh, Uint clusterLabel) {
  GradientColorMap<Uint, CMAP_JET> cmap_grad(0, clusterLabel); // watch out the interval boundary
  cmap_grad.addColor(Color::Red);
  cmap_grad.addColor(Color::Yellow);
  cmap_grad.addColor(Color::Blue);

  for (const auto& [key, voxel] : mapVoxel) {
    if (voxel.label == 0) {
      cout << "ERROR: " << voxel.p << " is " << voxel.visited << " and no label" << endl;
      continue;
    }
    Color cTemp = cmap_grad(voxel.label);

    // SDP color
    fs << voxel.p[0] << " " << voxel.p[1] << " " << voxel.p[2] << " " << int(cTemp.red()) << " " << int(cTemp.green())
       << " " << int(cTemp.blue()) << " " << endl;
    // Faces color
    for (auto f : voxel.faces) {
      aMesh.setFaceColor(f, cTemp);
    }
  }
}

void outputHueShader(ofstream& fs, HashMapVoxel& mapVoxel, RealMesh& aMesh, Uint clusterLabel) {
  HueShadeColorMap<Uint> aColorMap(0, clusterLabel);

  for (const auto& [key, voxel] : mapVoxel) {
    if (voxel.label == 0) {
#ifdef DEBUG
      cout << "DEBUG: " << voxel.p << " is " << voxel.visited << " and ignored with no label" << endl;
#endif
      continue;
    }
    Color cTemp = aColorMap(voxel.label);
    // SDP color
    fs << voxel.p[0] << " " << voxel.p[1] << " " << voxel.p[2] << " " << int(cTemp.red()) << " " << int(cTemp.green())
       << " " << int(cTemp.blue()) << endl;
    // Faces color
    for (auto f : voxel.faces) {
      aMesh.setFaceColor(f, cTemp);
    }
  }
}

void outputRandomShader(ofstream& fs, HashMapVoxel& mapVoxel, RealMesh& aMesh, Uint clusterLabel) {
  RandomColorMap aColorMap(0, clusterLabel);
  aColorMap.addColor(Color::Red);
  aColorMap.addColor(Color::Green);
  aColorMap.addColor(Color::Blue);
  aColorMap.addColor(Color::Yellow);
  aColorMap.addColor(Color::White);
  aColorMap.addColor(Color::Black);

  for (auto v : mapVoxel) {
    if (mapVoxel.at(v.first).label == 0) {
#ifdef DEBUG
      cout << "DEBUG: " << v.second.p << " is " << v.second.visited << " and igonred with no label" << endl;
#endif
      continue;
    }
    Color cTemp = aColorMap(mapVoxel.at(v.first).label);
    // SDP color
    fs << v.second.p[0] << " " << v.second.p[1] << " " << v.second.p[2] << " " << int(cTemp.red()) << " "
       << int(cTemp.green()) << " " << int(cTemp.blue()) << " " << endl;
    // Faces color
    for (auto f : v.second.faces) {
      aMesh.setFaceColor(f, cTemp);
    }
  }
}

void applyShaders(ofstream& fs, const string& filename, HashMapVoxel& mapVoxel, RealMesh& mesh, Uint clusterLabel,
                  MenuMode shaderMode) {
  fs.open(filename);

  switch (shaderMode) {
  case 0:
    printMessage("colored with Gradient shader map.", INFO);
    outputGradientShader(fs, mapVoxel, mesh, clusterLabel);
    break;
  case 1:
    printMessage("Colored with Hue shader map.", INFO);
    outputHueShader(fs, mapVoxel, mesh, clusterLabel);
    break;
  case 2:
    printMessage("Colored with Random shader map.", INFO);
    outputRandomShader(fs, mapVoxel, mesh, clusterLabel);
    break;
  default:
    printMessage("Invalid shader mode." + to_string(shaderMode), ERROR);
  }

  fs.close();
}

void assignOutputName(CLIMENU& menu) {
  if (menu.outputMesh.empty()) {
    int inputSuffix = menu.inputFile.length() - 4; // .obj or .off
    int extraSuffix = menu.extraData.length() - 4; // .dat
    string extraFileName = menu.extraData.substr(0, extraSuffix);
    if (menu.outputMode == 2)
      menu.outputMesh = extraFileName + "_ConfColored" + menu.inputFile.substr(inputSuffix);
    else if (menu.outputMode == 1)
      menu.outputMesh = extraFileName + "_AccColored" + menu.inputFile.substr(inputSuffix);
    else {
      printMessage("Invalid functionality and stop the program.", ERROR);
    }
  }
}

int main(int argc, char** argv) {
  // parse command line using CLI
  CLIMENU appMenu;
  appMenu.menuParse(argc, argv);

  RealMesh aMesh(true);
  aMesh << appMenu.inputFile; // read input mesh

  // 1) Map creation
  HashMapVoxel mapVoxel;
  vector<AccVoxel> voxelList = AccVoxelHelper::getAccVoxelsFromFile(appMenu.extraData);
  for (auto& v : voxelList) {
    KeyType hashValue = AccVoxelHelper::accumulationHash(v.p);
    mapVoxel[hashValue] = v;
  }

  // 2) Cluster generation
  Timer traverseTimer("traverseTimer");
  traverseTimer.start();

  vector<vector<Uint>> globalCluster; // store voxel hash values for each cluster
  Uint clusterLabel = 0;
  if (appMenu.outputMode == 2) {
    // maintain all but selected the voxel with the highest confidence
    printMessage("Segment based on confidence(ratio): ", INFO);
    priority_queue<AccVoxel, vector<AccVoxel>, CompareConfsAsc> globalPQ;
    computeConfidence(mapVoxel, appMenu.theta);
    processConfLabel(mapVoxel, globalPQ, clusterLabel, globalCluster, appMenu.theta);
  } else if (appMenu.outputMode == 1) {
    // maintain all but selected the voxel with the highest votes
    printMessage("Segment based on accumulation(value): ", INFO);
    // std::priority_queue<AccVoxel, std::vector<AccVoxel>, CompareAccAsc> globalPQ;
    std::priority_queue<AccVoxel> globalPQ;
    processAccLabel(mapVoxel, globalPQ, clusterLabel, globalCluster);
  } else
    printMessage("Invalid Mode", ERROR);

  traverseTimer.stop();
  traverseTimer.print();

  // 3) Shader faces associated SDP
  ofstream fout;
  if (appMenu.outputSDP.empty())
    appMenu.outputSDP = appMenu.extraData.substr(0, appMenu.extraData.length() - 4) + "_SDP.dat";
  applyShaders(fout, appMenu.outputSDP, mapVoxel, aMesh, clusterLabel, appMenu.shaderMode);
  printMessage("Export output in: " + appMenu.outputSDP, INFO);

  assignOutputName(appMenu);
  fout.open(appMenu.outputMesh);
  aMesh >> appMenu.outputMesh;
  printMessage("Export output in: " + appMenu.outputMesh, INFO);
  fout.close();

  return 0;
}
