#pragma once

#if defined(AccVoxelHelper_RECURSES)
#error Recursive header files inclusion detected in AccVoxelHelper.h
#else // defined(AccVoxelHelper_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AccVoxelHelper_RECURSES

#if !defined AccVoxelHelper_h
/** Prevents repeated inclusion of headers. */
#define AccVoxelHelper_h

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/PointListReader.h"

#include "AccVoxel.h"

/// @brief Defined in AccVoxelHelper.h
typedef size_t KeyType;

/// @brief A comparator class for selecting votes or confidence in AccVoxels
class AccComparator
{
public:
    bool (*compareFunction)(const AccVoxel &, const AccVoxel &);

    AccComparator(){};

    AccComparator(bool (*compare)(const AccVoxel &, const AccVoxel &))
    {
        compareFunction = compare;
    }

    bool compare(const AccVoxel &e1, const AccVoxel &e2)
    {
        return compareFunction(e1, e2);
    }
};

class AccVoxelHelper
{
public:
    /// @brief Read extracted file includes accumulation and confidence
    /// @param filename
    /// @return
    static std::vector<AccVoxel> getAccVoxelsFromFile(std::string filename)
    {
        typedef DGtal::PointVector<1, DGtal::int32_t> Point1D;
        std::vector<AccVoxel> resVectAcc;
        std::vector<unsigned int> indexFace;
        auto vOrigins = DGtal::PointListReader<Point1D>::getPolygonsFromFile(filename);
        for (auto l : vOrigins)
        {
            if (l.size() > 4)
            {
                AccVoxel::Point3D pt(l[0][0], l[1][0], l[2][0]);
                AccVoxel accV(pt, l[3][0], l[4][0], 0, false);
                for (unsigned int i = 5; i < l.size(); i++)
                {
                    accV.faces.push_back(l[i][0]);
                }
                resVectAcc.push_back(accV);
            }

#ifdef DEBUG
            cout << endl;
            cout << "P:" << accV.p << " | ";
            cout << "votes:" << accV.votes << " -> ";
            for (auto f : accV.faces)
            {
                cout << f << " ";
            }
            cout << endl;
#endif
        }

        return resVectAcc;
    }

    /// @brief A function to hash a point
    static KeyType hash(AccVoxel::Point3D p)
    {
        return std::hash<AccVoxel::Point3D>{}(p);
    }
};

bool compareVotesAsc(const AccVoxel &e1, const AccVoxel &e2)
{
    return e1.votes < e2.votes;
}

bool compareConfsAsc(const AccVoxel &e1, const AccVoxel &e2)
{
    return e1.confs < e2.confs;
}
#endif // !defined AccVoxelHelper_h

#undef AccVoxelHelper_RECURSES
#endif // else defined(AccVoxelHelper_RECURSES)
