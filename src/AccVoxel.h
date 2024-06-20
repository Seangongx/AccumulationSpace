#pragma once

#if defined(AccVoxel_RECURSES)
#error Recursive header files inclusion detected in AccVoxel.h
#else // defined(AccVoxel_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AccVoxel_RECURSES

#if !defined AccVoxel_h
/** Prevents repeated inclusion of headers. */
#define AccVoxel_h

#include <fstream>
#include <unordered_map>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

// @brief A structure to store a voxel in max-heap
class AccVoxel
{
public:
  typedef DGtal::uint32_t Uint;
  typedef DGtal::Z3i::Point Point3D; // Interger 3D point ( Z3i )

public:
  Point3D p;
  Uint votes;
  float confs;
  Uint label;
  bool visited;
  std::vector<Uint> faces;

  AccVoxel()
  {
    p = Point3D(0, 0, 0);
    votes = 0;
    confs = 0;
    label = 0;
    visited = false;
  }
  AccVoxel(Point3D _p, Uint _v, Uint _c, Uint _l, bool _f) : p(_p), votes(_v), confs(_c), label(_l), visited(_f) {}

  /// @brief Defualt compare operator for accumulation votes
  /// @param rhs
  /// @return
  bool operator<(const AccVoxel &rhs) const
  {
    return votes < rhs.votes;
  }
};

class FaceMap
{
private:
  std::unordered_map<size_t, std::set<size_t>> data; // mapping face index to hashValue of voxels
  std::ofstream logFile;                             // log file stream
  std::string logFileName{"faceMapLog.txt"};         // log file name

public:
  FaceMap()
  {
    logFile.open(logFileName, std::ios::out | std::ios::trunc); // Using trunc to overwrite
    if (!logFile)
    {
      std::cerr << "Unable to open log file: " << logFileName << std::endl;
    }
    else
    {
      logFile << "Log file opened successfully." << std::endl;
    }
  }

  ~FaceMap()
  {
    if (logFile.is_open())
    {
      logFile.close();
    }
  }

  // Add an empty set for the given index
  void addIndex(int index)
  {
    if (data.find(index) == data.end())
    {
      data[index] = std::set<size_t>();
      logFile << "Index " << index << " added." << std::endl;
    }
    else
    {
      logFile << "Index " << index << " already exists." << std::endl;
    }
  }

  // Erase the set of voxels for the given index
  void removeIndex(int index)
  {
    data.erase(index);
    logFile << "Index " << index << " removed." << std::endl;
  }

  // Add a voxel hash value to the set for the given index
  bool addValue(int index, size_t value)
  {
    if (data.find(index) == data.end())
    {
      logFile << "Index " << index << " not found, adding index." << std::endl;
      addIndex(index);
    }

    auto &values = data[index];
    auto result = values.insert(value);
    if (!result.second)
    {
      logFile << "Value " << value << " already exists in the set for index " << index << "." << std::endl;
      return false;
    }
    logFile << "Value " << value << " added to index " << index << "." << std::endl;
    return true;
  }

  // Remove a voxel hash value from the set for the given index
  void removeValue(int index, int value)
  {
    if (data.find(index) != data.end())
    {
      auto &values = data[index];
      size_t erased = values.erase(value);
      if (erased == 0)
      {
        logFile << "Value " << value << " not found in the set for index " << index << "." << std::endl;
      }
      else
      {
        logFile << "Value " << value << " removed from index " << index << "." << std::endl;
      }
    }
    else
    {
      logFile << "Index " << index << " not found." << std::endl;
    }
  }

  // Query the set of voxel hash values for the given index
  const std::set<size_t> *getValues(int index) const
  {
    auto it = data.find(index);
    if (it != data.end())
    {
      return &(it->second);
    }
    else
    {
      return nullptr;
    }
  }

  // Print all data to the log file
  void print()
  {
    for (const auto &pair : data)
    {
      logFile << "Index " << pair.first << ": ";
      for (size_t value : pair.second)
      {
        logFile << value << " ";
      }
      logFile << std::endl;
    }
  }
};

#endif // !defined AccVoxel_h

#undef AccVoxel_RECURSES
#endif // else defined(AccVoxel_RECURSES)
