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

#include "DGtal/base/Common.h"
// #include "DGtal/images/ImageContainerByHashTree.h"
#include <DGtal/images/ImageContainerBySTLMap.h>
#include "DGtal/io/readers/MeshReader.h"
#include "DGtal/io/writers/MeshWriter.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/helpers/StdDefs.h"
#include "CLI11.hpp"

///////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace DGtal;
///////////////////////////////////////////////////////////////////////////////
// #define DEBUG

/**
 @page segmentFromAcc segmentFromAcc

 @brief  find voxel neigbour with the accumulation value


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

/// @brief A structure to store a voxel in max-heap
class MyVoxel
{
public:
  Point3D p;
  Uint votes;
  bool visited;
  vector<Uint> faces;

  MyVoxel()
  {
    p = Point3D(0, 0, 0);
    votes = 0;
    visited = false;
  }
  MyVoxel(Point3D _p, Uint _v, bool _f) : p(_p), votes(_v), visited(_f) {}
  bool operator<(const MyVoxel &rhs) const
  {
    return votes < rhs.votes;
  }
};

typedef SpaceND<3> Space;
typedef HyperRectDomain<Space> Dom;
// typedef DGtal::ImageContainerBySTLMap<Dom, MyVoxel> HashMapVoxel;
typedef std::map<Uint, MyVoxel> HashMapVoxel; // Map id to voxel
// typedef std::multimap<Uint, Uint> HashMapV2F; // Map Point to faces

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

/// @brief A function to hash a point
Uint myHash(Point3D p)
{
  DGtal::uint64_t seed = 0;
  seed ^= std::hash<double>()(p[0]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<double>()(p[1]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<double>()(p[2]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
  // return seed % 100000;
}

/// @brief Find first unvisited voxel cordinates with the maximum votes
/// @param HashMapVoxel
/// @param Point3D
/// @return Point3D if true or Nothing if false;
uint getKeyWithMaxValue(const HashMapVoxel &voxelMap)
{
  uint maxKey = 0;
  uint maxValue = 0;
  // uint maxValue = std::numeric_limits<int>::min(); interesting error

  for (const auto &v : voxelMap)
  {
    if (v.second.visited)
      continue;
    if (v.second.votes > maxValue)
    {
      maxValue = v.second.votes;
      maxKey = v.first;
    }
  }
#ifdef DEBUG
  cout << "Max voxel is " << maxKey << " with " << maxValue << " votes " << endl;
#endif
  return maxKey;
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

int main(int argc, char **argv)
{

#pragma region 0) parse command line using ----------------------------------------------
  // parse command line using CLI ----------------------------------------------
  CLI::App app;
  std::string inputFileName;
  std::string colorFileName; // The file containing the index to be colored in the colors
  std::string outputFileName;

  // typedef DGtal::experimental::ImageContainerByHashTree<Dom, DGtal::Z3i::Point, DGtal::uint64_t> HashTreeVertex;

  app.description("Converts a .obj mesh into the .off format.\n"
                  "Typical use example:\n \t meshColorFromIndex -i file.obj -o file.off \n");
  app.add_option("-i,--input,1", inputFileName, "an input mesh file in .obj format.")
      ->required()
      ->check(CLI::ExistingFile);
  app.add_option("-c,--colorFile,2", colorFileName, "an input file containing index to colored.")
      ->required()
      ->check(CLI::ExistingFile);
  app.add_option("--output,-o,3", outputFileName, "an output file ");

  app.get_formatter()->column_width(40);
  CLI11_PARSE(app, argc, argv);
  // END parse command line using CLI ----------------------------------------------
#pragma endregion

  // read input mesh
  DGtal::Mesh<DGtal::Z3i::RealPoint> aMesh(true);
  aMesh << inputFileName;
  std::vector<unsigned int> indexFace;
  auto vOrigins = DGtal::PointListReader<Point1D>::getPolygonsFromFile(colorFileName);
  auto bbox = getBoundingBox(vOrigins);
  std::cout << "bounding box" << bbox.first << " " << bbox.second << std::endl;

#pragma region 1) Image creation
  // domain creation:
  Dom aDomain(bbox.first, bbox.second);
  // HashMapVoxel mapVoxel(aDomain);
  HashMapVoxel mapVoxel;

  //   HashMapV2F mapV2T;
  // store data:
  for (auto l : vOrigins)
  {
    Point3D p(l[0][0], l[1][0], l[2][0]);
    Uint hashValue = myHash(p);
    MyVoxel v(p, l[3][0], false);
    for (int it = 4; it < l.size(); it++)
    {
      v.faces.push_back(l[it][0]);
    }

#ifdef DEBUG
    cout << endl;
    cout << "P:" << p[0] << " " << p[1] << " " << p[2] << " | ";
    cout << "votes:" << l[3][0] << " | ";
    cout << hashValue
         << " -> ";
    for (auto f : v.faces)
    {
      cout << f << " ";
    }
    cout << endl;
#endif

    // add to mapVoxel
    mapVoxel[hashValue] = v;
  }
#pragma endregion

#pragma region 2) traverse the accumulation image

  std::priority_queue<MyVoxel> globalPQ; // starting from the voxel with the highest votes:
  std::queue<MyVoxel> localQ;            // starting from the voxel currently visited:
  uint count = 0;

  // add all voxels to the global priority queue
  for (auto v : mapVoxel)
  {
    globalPQ.push(v.second);
  }
  cout << "Global queue size is " << globalPQ.size() << endl;

  // LOOP I: select the next voxel with the highest votes
  while (!globalPQ.empty())
  {
    auto vCurrent = globalPQ.top();
    auto pCurrent = vCurrent.p;
    auto idCurrent = myHash(vCurrent.p);

    if (mapVoxel.at(idCurrent).visited)
    {
      globalPQ.pop();
      continue;
    }

    // auto keyCurrent = getKeyWithMaxValue(mapVoxel);
    // cout << "Current voxel is : " << idCurrent << " and calculate is: " << keyCurrent << endl;

    // LOOP II: traverse the adjacent voxels
    int gridStep = 1;
    for (int dx = -gridStep; dx <= gridStep; ++dx)
    {
      for (int dy = -gridStep; dy <= gridStep; ++dy)
      {
        for (int dz = -gridStep; dz <= gridStep; ++dz)
        {
          Point3D pAdjacent(vCurrent.p[0] + dx, vCurrent.p[1] + dy, vCurrent.p[2] + dz);
          auto idAdjacent = myHash(pAdjacent);
          if (mapKeychecker(mapVoxel, idAdjacent))
          {
            // if (isOnBoundary(pAdjacent, bbox)) // skip the boundary voxel
            //   continue;
            if (mapVoxel.at(idAdjacent).visited) // skip the visited voxel
              continue;
            if (pAdjacent == vCurrent.p) // skip the center itself
              continue;

            // Paint the associated faces
            DGtal::Color color(pCurrent[0] % 255, pCurrent[1] % 255, pCurrent[2] % 255);
            for (auto f : mapVoxel.at(idAdjacent).faces)
              aMesh.setFaceColor(f, color);
            cout << count++ << ": Visisting " << idAdjacent << " : " << pAdjacent << endl;
            cout << "-----------------------------------------------------" << endl;
          }
        }
      }
    }

    // mark visited
    mapVoxel.at(myHash(vCurrent.p)).visited = true;
    cout << globalPQ.size() << endl;
  }

#pragma endregion

  //   // for (auto l : vOrigins)
  //   // {
  //   //   unsigned int k = 0;
  //   //   for (auto i : l)
  //   //   {
  //   //     if (k > 3)
  //   //     {
  //   //       aMesh.setFaceColor(i[0], DGtal::Color::Red);
  //   //     }
  //   //     k++;
  //   //   }
  //   // }

  ofstream fout;
  outputFileName = inputFileName + "_colored.off";
  fout.open(outputFileName);
  cout << "Writing output file " << outputFileName << endl;
  MeshWriter<DGtal::Z3i::RealPoint>::export2OFF(fout, aMesh);
  return 0;
}
