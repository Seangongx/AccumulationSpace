#ifndef DENOISE_H
#define DENOISE_H

#include <string>
#include <utility>
#include "Curvature.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/PolygonalSurface.h"

typedef DGtal::Z3i::RealPoint RealPoint;
typedef DGtal::PolygonalSurface<RealPoint> PolySurface;
typedef DGtal::HalfEdgeDataStructure HFEdge;

static double cotan_weight(const PolySurface& mesh, PolySurface::Arc arc) {

  double weight = 0.0;
  // 获取半边
  HFEdge::HalfEdge h1 = mesh.heds().halfEdge(arc);
  auto arcOpposite = h1.opposite;
  HFEdge::HalfEdge h2 = mesh.heds().halfEdge(arcOpposite);
  // 获取arc的两个端点
  const RealPoint p1 = mesh.position(mesh.head(arc));
  const RealPoint p2 = mesh.position(mesh.tail(arc));
  if (!mesh.isArcBoundary(arc)) {
    const RealPoint p3 = mesh.position(h1.toVertex);
    const RealPoint d0 = p1 - p3;
    const RealPoint d1 = p2 - p3;
    const double area = (d0.crossProduct(d1)).norm();
    if (area > std::numeric_limits<double>::epsilon()) {
      const double cot = d0.dot(d1) / area;
      weight += std::max(-3.0, std::min(3.0, cot));  // clamp_cot
    }
  }
  if (!mesh.isArcBoundary(arcOpposite)) {
    const RealPoint p3 = mesh.position(h2.toVertex);
    const RealPoint d0 = p1 - p3;
    const RealPoint d1 = p2 - p3;
    const double area = (d0.crossProduct(d1)).norm();

    if (area > std::numeric_limits<double>::epsilon()) {
      const double cot = d0.dot(d1) / area;
      weight += std::max(-3.0, std::min(3.0, cot));  // clamp_cot
    }
  }

  assert(!std::isnan(weight));
  assert(!std::isinf(weight));

  auto arcVertex = mesh.heds().arcFromHalfEdgeIndex(arc);
  const RealPoint p3 = mesh.position(arcVertex.first);
  const RealPoint p4 = mesh.position(arcVertex.second);
  assert(p1 == p4);
  assert(p2 == p3);

  return weight;
}

class DenoiseSurface {
 public:
  DenoiseSurface(PolySurface& _mesh) : mesh(_mesh) {
    eWeight =
        PolySurface::IndexedPropertyMap<double>(mesh, mesh.allArcs().size());
    vLaplace =
        PolySurface::IndexedPropertyMap<RealPoint>(mesh, mesh.nbVertices());
    for (const auto& e : mesh.allArcs())
      eWeight[e] = std::max(0.1, cotan_weight(mesh, e));
  }
  ~DenoiseSurface() = default;

  RealPoint boundary_center() {
    RealPoint center(0, 0, 0);
    for (auto vBoundary : mesh.heds().boundaryVertices())
      center += mesh.position(vBoundary);
    return center / mesh.heds().boundaryVertices().size();
  }

  double boundary_std() {
    RealPoint center = boundary_center();
    double std_ = 0;
    for (auto vBoundary : mesh.heds().boundaryVertices())
      std_ += pow((mesh.position(vBoundary) - center).norm(), 2);

    return std::sqrt(std_ / mesh.heds().boundaryVertices().size());
  }

  void boundary_explicit_iterate(float lambda) {
    // 缓存边界顶点
    std::vector<PolySurface::Vertex> boundaryVerts;
    for (auto v : mesh.heds().boundaryVertices())
      boundaryVerts.push_back(v);

    if (boundaryVerts.empty())
      return;

    // 记录原始边界中心和标准差
    RealPoint prev_center(0, 0, 0);
    for (auto v : boundaryVerts)
      prev_center += mesh.position(v);
    prev_center /= boundaryVerts.size();

    double prev_std = 0;
    for (auto v : boundaryVerts)
      prev_std += pow((mesh.position(v) - prev_center).norm(), 2);
    prev_std = std::sqrt(prev_std / boundaryVerts.size());

    // 计算拉普拉斯向量
    for (auto v : boundaryVerts) {
      double w = 0;
      vLaplace[v] = RealPoint(0, 0, 0);
      for (auto arc : mesh.outArcs(v)) {
        auto vv = mesh.head(arc);
        if (!mesh.isVertexBoundary(vv))
          continue;
        w += eWeight[arc];
        vLaplace[v] += eWeight[arc] * (mesh.position(vv) - mesh.position(v));
      }
      if (w > std::numeric_limits<double>::epsilon())
        vLaplace[v] /= w;
      else
        vLaplace[v] = RealPoint(0, 0, 0);
    }

    // 更新边界顶点位置
    for (auto v : boundaryVerts)
      mesh.position(v) += lambda * vLaplace[v];

    // 以原中心为基准缩放
    RealPoint curr_center(0, 0, 0);
    for (auto v : boundaryVerts)
      curr_center += mesh.position(v);
    curr_center /= boundaryVerts.size();

    double curr_std = 0;
    for (auto v : boundaryVerts)
      curr_std += pow((mesh.position(v) - curr_center).norm(), 2);
    curr_std = std::sqrt(curr_std / boundaryVerts.size());

    double scale = prev_std / curr_std;
    for (auto v : boundaryVerts)
      mesh.position(v) = prev_center + scale * (mesh.position(v) - prev_center);

    // 缩放后重新计算当前中心
    RealPoint new_center(0, 0, 0);
    for (auto v : boundaryVerts)
      new_center += mesh.position(v);
    new_center /= boundaryVerts.size();

    RealPoint t = prev_center - new_center;
    for (auto v : boundaryVerts)
      mesh.position(v) += t;
  }

  void explicitIterate(float lambda, bool boundarySmoothing) {
    if (boundarySmoothing)
      boundary_explicit_iterate(lambda);

    // Calculate the Laplace operator for each vertex
    for (const auto& v : mesh.allVertices()) {
      if (mesh.isVertexBoundary(v))
        continue;
      double w = 0;
      vLaplace[v] = RealPoint(0, 0, 0);
      for (const auto& arc : mesh.outArcs(v)) {
        w += eWeight[arc];
        PolySurface::Vertex outv = mesh.head(arc);
        vLaplace[v] += eWeight[arc] * (mesh.position(outv) - mesh.position(v));
        assert(v == mesh.tail(arc));
      }
      if (w > std::numeric_limits<double>::epsilon())
        vLaplace[v] /= w;
      else
        vLaplace[v] = RealPoint(0, 0, 0);  // 或者跳过
    }

    // Apply the Laplace operator to the vertices
    for (const auto& v : mesh.allVertices()) {
      if (!mesh.isVertexBoundary(v))
        mesh.position(v) += lambda * vLaplace[v];
    }
  }

 private:
  PolySurface& mesh;
  size_t n_inner_vertices_{0};
  size_t n_boundary_vertices_{0};

  // property handles
  PolySurface::IndexedPropertyMap<double> eWeight;      // edge weight
  PolySurface::IndexedPropertyMap<RealPoint> vLaplace;  // vertex Laplace
  SurfaceCurvature sCurvature;
  //Eigen::SparseMatrix<double> implicit_L;
};

#endif  //DENOISE_H