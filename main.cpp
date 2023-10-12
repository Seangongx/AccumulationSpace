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

/// @brief A structure to store the vote of a voxel
class VoteVoxel
{
public:
  VoteVoxel()
  {
    p = Z3i::Point(0, 0, 0);
    votes = 0;
  }
  VoteVoxel(Z3i::Point p, uint32_t votes) : p(p), votes(votes) {}
  Z3i::Point p;
  uint32_t votes;
  bool operator<(const VoteVoxel &rhs) const
  {
    return votes < rhs.votes;
  }
};

// Point include 1 dimension
typedef PointVector<1, DGtal::int32_t> Point1D;
typedef SpaceND<3> Space;
typedef HyperRectDomain<Space> Dom;
// typedef DGtal::ImageContainerBySTLMap<Dom, bool> HashMapFlag;           // Flag
// typedef DGtal::ImageContainerBySTLMap<Dom, DGtal::int32_t> HashMapAccv; // Accumulation Value
typedef std::map<DGtal::uint32_t, bool> HashMapFlag;           // Flag
typedef std::map<DGtal::uint32_t, VoteVoxel> HashMapAccv;      // Accumulation Value
typedef std::multimap<Z3i::Point, DGtal::uint32_t> HashMapP2F; // Point to faces

std::pair<Z3i::Point, Z3i::Point>
getBoundingBox(std::vector<std::vector<Point1D>> lData)
{
  std::pair<Z3i::Point, Z3i::Point> result(Z3i::Point(std::numeric_limits<int>::max(),
                                                      std::numeric_limits<int>::max(),
                                                      std::numeric_limits<int>::max()),
                                           Z3i::Point(std::numeric_limits<int>::min(),
                                                      std::numeric_limits<int>::min(),
                                                      std::numeric_limits<int>::min()));
  for (auto lP : lData)
  {
    Z3i::Point p(lP[0][0], lP[1][0], lP[2][0]);
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
DGtal::uint32_t
myHash(Z3i::Point p)
{
  DGtal::uint64_t seed = 0;
  seed ^= std::hash<double>()(p[0]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<double>()(p[1]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<double>()(p[2]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
  // return seed % 100000;
}

/// @brief Find next unvisited max key
/// @param accMap
/// @param flagMap
/// @return interger(key) or -1(no unvisited key)
DGtal::uint32_t
getKeyWithMaxValue(const HashMapAccv &accMap, const HashMapFlag &flagMap)
{
  int32_t maxValue = 0; // minimum value
  int32_t maxKey = -1;  // unvalid key
  // int32_t maxValue = std::numeric_limits<int>::min(); interesting error

  for (const auto &pair : accMap)
  {
    // if flag in flagmap is true(visited), then skip
    if (flagMap.at(pair.first))
      continue;
    if (pair.second.votes > maxValue)
    {
      maxValue = pair.second.votes;
      maxKey = pair.first;
    }
  }
#ifdef DEBUG
  cout << "Max key is " << maxKey << " at point " << accMap.at(maxKey).p
       << " with " << maxValue << " votes " << endl;
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

bool isOnBoundary(const Z3i::Point &p, std::pair<Z3i::Point, Z3i::Point> bbox)
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
  std::string outputFileName = inputFileName + "_colored.off";

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
  // HashMapFlag imageFlag(aDomain);
  // HashMapAccv imageAccv(aDomain);
  HashMapFlag imageFlag;
  HashMapAccv imageAccv;
  HashMapP2F mapP2T;
  // store data:
  for (auto l : vOrigins)
  {
    Z3i::Point p(l[0][0], l[1][0], l[2][0]);
    DGtal::uint32_t hashValue = myHash(p);
    VoteVoxel vv(p, l[3][0]);
    imageFlag[hashValue] = false;
    imageAccv[hashValue] = vv;

#ifdef DEBUG
    cout << endl;
    cout << "P:" << p[0] << " " << p[1] << " " << p[2] << " | ";
    cout << "votes:" << l[3][0] << " | ";
    cout << hashValue << " -> ";

#endif
    for (int it = 4; it < l.size(); it++)
    {
      mapP2T.insert(std::make_pair(p, l[it][0]));

#ifdef DEBUG
      cout << l[it][0] << " ";
#endif
    }
  }
#pragma endregion

  // 2) traverse the accumulation image
  // starting from the vexel with the highest votes:
  std::priority_queue<VoteVoxel> vPQ;
  auto keyCurrent = getKeyWithMaxValue(imageAccv, imageFlag);

  // LOOP I: select the next voxel with the highest votes
  while (keyCurrent > 0)
  {
    cout << "In " << keyCurrent << " loop While:" << endl;

    // // 结束条件有点问题需要确认
    // if (!mapKeychecker(imageAccv, keyCurrent))
    // {
    //   keyCurrent = -1;
    //   break;
    // }

    // LOOP II: traverse the adjacent voxels
    vPQ.push(imageAccv.at(keyCurrent));
    while (!vPQ.empty())
    {
      VoteVoxel vCurrent = vPQ.top();
      vPQ.pop();
      auto idCurrent = myHash(vCurrent.p);
      cout << "The current center is : " << idCurrent << " : " << vCurrent.p << endl;

      // CHECK if the voxel is visited
      if (!mapKeychecker(imageFlag, idCurrent))
        continue;
      imageFlag[idCurrent] = true;

      for (int dx = -1; dx <= 1; ++dx)
      {
        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dz = -1; dz <= 1; ++dz)
          {
            Z3i::Point pAdjacent(vCurrent.p[0] + dx, vCurrent.p[1] + dy, vCurrent.p[2] + dz);
            if (pAdjacent == vCurrent.p) // skip the center itself
              continue;
            auto idAdjacent = myHash(pAdjacent);
            DGtal::Color color(pAdjacent[0] % 255, pAdjacent[1] % 255, pAdjacent[2] % 255);
            if (mapKeychecker(imageAccv, idAdjacent) && mapKeychecker(imageFlag, idAdjacent))
            {
              if (imageFlag.at(idAdjacent) == true)
                continue;
              vPQ.push(imageAccv[idAdjacent]);
              cout << "Push " << idAdjacent << pAdjacent << " with " << imageFlag[idAdjacent] << endl;

              // color the assocaited faces, but why loop last one twice ？？？
              // Todo: check the loop
              auto range = mapP2T.equal_range(pAdjacent);
              for (auto it = range.first; it != range.second; ++it)
                aMesh.setFaceColor(it->second, color);
              cout << "-----------------------------------------------------" << endl;
            }
          }
        }
      }

      // if (x < gridWidth - 1)
      // {
      //   pq.push({x + 1, y, z, grid[x + 1][y][z]});
      // }
      // 添加其他相邻体素的逻辑
      cout << "Next round will pop a new candidate: " << endl;
    }

    auto keyCurrent = getKeyWithMaxValue(imageAccv, imageFlag);
    cout << "Current queue empty and try to find a second maximum key " << keyCurrent << endl;
  }

  // for (auto l : vOrigins)
  // {
  //   unsigned int k = 0;
  //   for (auto i : l)
  //   {
  //     if (k > 3)
  //     {
  //       aMesh.setFaceColor(i[0], DGtal::Color::Red);
  //     }
  //     k++;
  //   }
  // }

  ofstream fout;
  fout.open(outputFileName);
  MeshWriter<DGtal::Z3i::RealPoint>::export2OFF(fout, aMesh);
  return 0;
}
