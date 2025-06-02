#ifndef CURVATURE_H
#define CURVATURE_H

#include "DGtal/geometry/meshes/NormalCycleComputer.h"
#include "DGtal/shapes/PolygonalSurface.h"

typedef DGtal::Z3i::RealPoint RealPoint;
typedef DGtal::Z3i::RealVector RealVector;
typedef DGtal::PolygonalSurface<RealPoint> PolySurface;
typedef DGtal::SurfaceMesh<RealPoint, RealVector> SM;
typedef DGtal::NormalCycleComputer<RealPoint, RealVector> NC;

class SurfaceCurvature {
 public:
  //! Construct with mesh to be processed.
  SurfaceCurvature() {}

  // destructor
  ~SurfaceCurvature() {}

  PolySurface::IndexedPropertyMap<double>& update_gauss_curvature();
  PolySurface::IndexedPropertyMap<double>& update_mean_curvature();

  double max_gauss_curvature();
  double max_mean_curvature();

 private:
  PolySurface::IndexedPropertyMap<double> mean_curvature_;
  PolySurface::IndexedPropertyMap<double> gauss_curvature_;

  double gauss_kmax_{-1};
  double mean_kmax_{-1};
};

#endif