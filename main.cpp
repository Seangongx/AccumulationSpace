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
 * @author Xun Gong (\c xun.gong@telecom-paris.fr )
 *
 * @date 2023/10/06
 *
 * Source file of the tool segmentFromAcc
 *
 * This file is part of the DGtal library/DGtalTools-contrib Project.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <cstddef>

#include "DGtal/base/Common.h"
// #include "DGtal/images/ImageContainerByHashTree.h"
#include <DGtal/images/ImageContainerBySTLMap.h>
#include "DGtal/io/readers/MeshReader.h"
#include "DGtal/io/writers/MeshWriter.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/helpers/StdDefs.h"
#include "CLI11.hpp"
#include "AccVoxel.h"
#include "AccVoxelHelper.h"
#include <DGtal/io/colormaps/GradientColorMap.h>

///////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace DGtal;
///////////////////////////////////////////////////////////////////////////////
// #define DEBUG

/**
 @page segmentFromAcc segmentFromAcc

 @brief get accumulation or confidence clusters based on the accumulation image and
  color the associate segement on the original mesh.


 @code
 Typical use example:
     segmentFromAcc -i file.obj -o file.off

 Usage: ./expe/segmentFromAcc [OPTIONS] 1 [2]

 Positionals:
   1 TEXT:FILE REQUIRED                  an input mesh file in .obj format.
   2 TEXT                                an output file

 Options:
   -h,--help                             Print this help message and exit
   -i,--input TEXT:FILE REQUIRED         an input mesh file in .obj format.
   -o,--output TEXT                      an output file


 @endcode

 @b Example:

 @code
 segmentFromAcc $DGtal/examples/samples/am_beech2-3.off am_beech2-3Acc.dat
 @endcode

 @see
 @ref segmentFromAcc.cpp

 */

#pragma region Type Definitions

typedef DGtal::uint32_t Uint;
typedef DGtal::Z3i::Point Point3D;                     // Interger 3D point ( Z3i )
typedef DGtal::PointVector<1, DGtal::int32_t> Point1D; // Point include 1 dimension

typedef SpaceND<3> Space;
typedef HyperRectDomain<Space> Dom;
typedef std::map<Uint, AccVoxel> HashMapVoxel; // Map id to voxel
// typedef DGtal::ImageContainerBySTLMap<Dom, MyVoxel> HashMapVoxel;

#pragma endregion

std::pair<Point3D, Point3D>
getBoundingBox(std::vector<std::vector<Point1D>> lData)
{
  std::pair<Point3D, Point3D> result(Point3D(std::numeric_limits<int>::max(),
                                             std::numeric_limits<int>::max(),
                                             std::numeric_limits<int>::max()),
                                     Point3D(std::numeric_limits<int>::min(),
                                             std::numeric_limits<int>::min(),
                                             std::numeric_limits<int>::min()));
  for (auto lP : lData)
  {
    Point3D p(lP[0][0], lP[1][0], lP[2][0]);
    for (int i = 0; i < 3; i++)
    {
      if (result.first[i] > p[i])
      {
        result.first[i] = p[i];
      }
      if (result.second[i] < p[i])
      {
        result.second[i] = p[i];
      }
    }
  }

  return result;
}

/// @brief Check if a key is in a map
/// @tparam MapType
/// @tparam KeyType
/// @param map
/// @param key
/// @return true(exist) or false(not exist)
template <typename MapType, typename KeyType>
bool mapKeychecker(const MapType &map, const KeyType &key)
{
  auto it = map.find(key);
  if (it != map.end())
    return true;
  return false;
}

bool isOnBoundary(const Point3D &p, std::pair<Point3D, Point3D> bbox)
{
  for (int i = 0; i < 3; i++)
  {
    if (p[i] == bbox.first[i] || p[i] == bbox.second[i])
    {
      return true;
    }
  }
  return false;
}

/// @brief When a voxel is unvisited without label, group it in current cluster
/// @param voxel
/// @param mapVoxel
/// @param queue
void labelNeighbours(const AccVoxel &voxel, HashMapVoxel &mapVoxel, std::queue<AccVoxel> &queue)
{
#ifdef DEBUG
  cout << "Current center is " << voxel.label
       << " -> " << voxel.p << endl;
#endif

  int gridStep = 1;
  for (int dx = -gridStep; dx <= gridStep; ++dx)
  {
    for (int dy = -gridStep; dy <= gridStep; ++dy)
    {
      for (int dz = -gridStep; dz <= gridStep; ++dz)
      {
        // locate the adjacent voxel id
        Point3D pTemp(voxel.p[0] + dx, voxel.p[1] + dy, voxel.p[2] + dz);
        auto idTemp = AccVoxelHelper::hash(pTemp);

        if (mapKeychecker(mapVoxel, idTemp))
        {
          // if (isOnBoundary(pAdjacent, bbox)) // skip the boundary voxel
          //   continue;
          if (mapVoxel.at(idTemp).visited) // skip the visited voxel
            continue;
          if (pTemp == voxel.p) // skip the center itself
            continue;
          if (mapVoxel.at(idTemp).label == 0)
          {
            mapVoxel.at(idTemp).label = voxel.label;
            queue.push(mapVoxel.at(idTemp));

#ifdef DEBUG
            cout << "Label in [" << voxel.label << "] and push " << mapVoxel.at(idTemp).p << endl;
#endif
          }
        }
      }
    }
  }
}

template <typename TypePQ>
void processLabel(HashMapVoxel &mapVoxel, TypePQ &globalPQ, uint &clusterLabel)
{
  std::queue<AccVoxel> localQ; // maintain the local visited:

  for (auto v : mapVoxel)
    cout << v.first << " " << v.second.p << " " << v.second.confs << endl;
  for (auto v : mapVoxel)
    globalPQ.push(v.second);
  cout << "Loaded global queue size is " << globalPQ.size() << endl;

  // LOOP I: select the next voxel with the highest votes
  while (!globalPQ.empty())
  {
    auto vCurrent = globalPQ.top();
    auto pCurrent = vCurrent.p;
    auto idCurrent = AccVoxelHelper::hash(vCurrent.p);

    if (mapVoxel.at(idCurrent).visited)
    {
      globalPQ.pop();
#ifdef DEBUG
      cout << "Pop(G): " << pCurrent << " and remain " << globalPQ.size() << endl;
#endif
      continue;
    }

    // LOOP II: traverse the adjacent voxels

    // Select a new cluster starting voxel update the current cluster label
    mapVoxel.at(idCurrent).visited = true;
    mapVoxel.at(idCurrent).label = clusterLabel++;
    vCurrent.label = clusterLabel;

#ifdef DEBUG
    cout << "Current cluster center is " << pCurrent
         << " -> " << vCurrent.label << endl;
    cout << "Remain(G): " << globalPQ.size() << endl;
#endif

    labelNeighbours(vCurrent, mapVoxel, localQ);
    while (!localQ.empty())
    {
      auto vAdjacent = localQ.front();
      auto pAdjacent = vAdjacent.p;
      auto idAdjacent = AccVoxelHelper::hash(vAdjacent.p);

#ifdef DEBUG
      cout << "Visiting [" << clusterLabel << "] : " << pAdjacent << endl;
      cout << "Remain(L): " << localQ.size() << endl;
#endif

      if (mapVoxel.at(idAdjacent).visited)
      {
        localQ.pop();
        continue;
      }
      if (mapVoxel.at(idAdjacent).label == 0)
      {
        cout << "ERROR: " << pAdjacent << " is not visited but has no label" << endl;
      }

      // mark visited and update the current cluster label
      // mapVoxel.at(idAdjacent).label = clusterLabel;
      // vAdjacent.label = clusterLabel;
      mapVoxel.at(idAdjacent).visited = true;
      localQ.pop();

      labelNeighbours(vAdjacent, mapVoxel, localQ);
    }
  }

  cout << "Found " << clusterLabel << " clusters" << endl
       << "---------------------------------------------------------------------" << endl;
}

int main(int argc, char **argv)
{

#pragma region 0) parse command line using ----------------------------------------------
  // parse command line using CLI ----------------------------------------------
  CLI::App app;
  std::string inputFileName;
  std::string colorFileName;
  std::string outputFileName;
  char outputMode = 0;

  // typedef DGtal::experimental::ImageContainerByHashTree<Dom, DGtal::Z3i::Point, DGtal::uint64_t> HashTreeVertex;

  app.description("Converts a .obj mesh into the .off format.\n"
                  "Typical use example:\n \t meshColorFromIndex -i file.obj -o file.off \n");
  app.add_option("-i,--input,1", inputFileName, "an input mesh file in .obj format.")
      ->required()
      ->check(CLI::ExistingFile);
  app.add_option("-c,--colorFile,2", colorFileName, "an input file containing accumulation, confidence and index to colored.")
      ->required()
      ->check(CLI::ExistingFile);
  app.add_option("-o,--output,3", outputFileName, "an output file ");
  app.add_option("--outputMode,4", outputMode, "0 is for accmulation compare, 1 is for confidence compare");

  app.get_formatter()
      ->column_width(40);
  CLI11_PARSE(app, argc, argv);
  // END parse command line using CLI ----------------------------------------------
#pragma endregion

  // read input mesh
  DGtal::Mesh<DGtal::Z3i::RealPoint> aMesh(true);
  aMesh << inputFileName;
  // auto vOrigins = DGtal::PointListReader<Point1D>::getPolygonsFromFile(colorFileName);
  // auto bbox = getBoundingBox(vOrigins);
  // std::cout << "bounding box" << bbox.first << " " << bbox.second << std::endl;
  // // domain creation:
  // Dom aDomain(bbox.first, bbox.second);

#pragma region 1) Map creation
  // HashMapVoxel mapVoxel(aDomain);
  // HashMapV2F mapV2T;
  HashMapVoxel mapVoxel;

  // store voxels in the map:
  std::vector<AccVoxel> voxelList = AccVoxelHelper::getAccVoxelsFromFile(colorFileName);
  for (auto &v : voxelList)
  {
    KeyType hashValue = AccVoxelHelper::hash(v.p);
    mapVoxel[hashValue] = v;
  }
#pragma endregion

#pragma region 2) traverse the accumulation image

  uint clusterLabel = 0;
  if (outputMode == 1)
  {
    // AccComparator cmp(compareConfsAsc);
    // maintain all but selected the voxel with the highest confidence
    std::priority_queue<AccVoxel, std::vector<AccVoxel>, decltype(&compareConfsAsc)> globalPQ;
    processLabel(mapVoxel, globalPQ, clusterLabel);
  }
  else
  {
    // maintain all but selected the voxel with the highest votes
    std::priority_queue<AccVoxel> globalPQ;
    processLabel(mapVoxel, globalPQ, clusterLabel);
  }

#pragma endregion

#pragma region 3) color the mesh
  GradientColorMap<int> cmap_grad(0, clusterLabel);
  cmap_grad.addColor(Color(0, 0, 255));
  cmap_grad.addColor(Color(255, 0, 0));
  cmap_grad.addColor(Color(255, 255, 0));

  for (auto v : mapVoxel)
  {
    for (auto f : v.second.faces)
    {
      aMesh.setFaceColor(f, cmap_grad(mapVoxel.at(v.first).label));
    }
  }

#pragma endregion

  ofstream fout;
  if (outputFileName.empty())
  {
    if (outputMode == 1)

      outputFileName = colorFileName.substr(0, inputFileName.length() - 4) + "_ConfColored.off";
    else
      outputFileName = colorFileName.substr(0, inputFileName.length() - 4) + "_AccColored.off";
  }
  fout.open(outputFileName);
  cout << "Export output in: " << outputFileName << endl;
  MeshWriter<DGtal::Z3i::RealPoint>::export2OFF(fout, aMesh);
  return 0;
}
