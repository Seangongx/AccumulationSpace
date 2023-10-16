#pragma once

#if defined(AccVoxel_RECURSES)
#error Recursive header files inclusion detected in AccVoxel.h
#else // defined(AccVoxel_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AccVoxel_RECURSES

#if !defined AccVoxel_h
/** Prevents repeated inclusion of headers. */
#define AccVoxel_h

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

// @brief A class to store a voxel in max-heap
class AccVoxel
{
public:
  typedef DGtal::uint32_t Uint;
  typedef DGtal::Z3i::Point Point3D; // Interger 3D point ( Z3i )

public:
  Point3D p;
  Uint votes;
  bool visited;
  std::vector<Uint> faces;

  AccVoxel()
  {
    p = Point3D(0, 0, 0);
    votes = 0;
    visited = false;
  }
  AccVoxel(Point3D _p, Uint _v, bool _f) : p(_p), votes(_v), visited(_f) {}
  bool operator<(const AccVoxel &rhs) const
  {
    return votes < rhs.votes;
  }
};

#endif // !defined AccVoxel_h

#undef AccVoxel_RECURSES
#endif // else defined(AccVoxel_RECURSES)
