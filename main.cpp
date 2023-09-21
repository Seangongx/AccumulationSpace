#include "DGtal/shapes/TriangulatedSurface.h"
#include "DGtal/shapes/Mesh.h"
#include "DGtal/shapes/MeshHelpers.h"
#include "DGtal/io/viewers/Viewer3D.h"

using namespace std;
using namespace DGtal;
using namespace DGtal::Z3i;

int main(int argc, char **argv)
{
  typedef TriangulatedSurface<RealPoint> TriMesh;
  typedef Mesh<RealPoint> ViewMesh;

  // Creates two triangles glued together.
  TriMesh tmesh;
  tmesh.addVertex(RealPoint(0, 0, 0));
  tmesh.addVertex(RealPoint(1, 0, 0));
  tmesh.addVertex(RealPoint(0, 1, 0));
  tmesh.addVertex(RealPoint(1, 1, 1));
  tmesh.addTriangle(0, 1, 2);
  tmesh.addTriangle(2, 1, 3);
  tmesh.build();

  // Convert it to a mesh
  ViewMesh mesh;
  MeshHelpers::triangulatedSurface2Mesh(tmesh, mesh);

  // View it
  QApplication application(argc, argv);
  Viewer3D<> viewer;
  viewer.show();
  viewer.setLineColor(Color(150, 0, 0, 254));
  viewer << mesh;
  viewer << Viewer3D<>::updateDisplay;
  application.exec();

  return 0;
}

// QApplication application(argc,argv);
// Viewer3D<> viewer;
// viewer.show();

// // A mesh is constructed and faces are added from the vertex set.
// Mesh<Point> aMesh(true);

// aMesh.addVertex(Point(0,0,0));
// aMesh.addVertex(Point(1,0,0));
// aMesh.addVertex(Point(1,1,0));

// aMesh.addVertex(Point(0,0,1));
// aMesh.addVertex(Point(1,0,1));
// aMesh.addVertex(Point(1,1,1));
// aMesh.addVertex(Point(0,1,1));

// aMesh.addVertex(Point(0,1,0));
// aMesh.addVertex(Point(0,2,0));
// aMesh.addVertex(Point(0,3,1));
// aMesh.addVertex(Point(0,2,2));
// aMesh.addVertex(Point(0,1,2));
// aMesh.addVertex(Point(0,0,1));
// aMesh.addTriangularFace(0, 1, 2, Color(150,0,150,104));
// aMesh.addQuadFace(6,5,4,3, Color::Blue);

// Mesh<Point>::MeshFace listIndex;
// listIndex.push_back(7);
// listIndex.push_back(8);
// listIndex.push_back(9);
// listIndex.push_back(10);
// listIndex.push_back(11);
// listIndex.push_back(12);
// aMesh.addFace(listIndex, Color(150,150,0,54));

// viewer.setLineColor(Color(150,0,0,254));
// viewer << aMesh;
// viewer << Viewer3D<>::updateDisplay;
// bool res = application.exec();
// FATAL_ERROR(res);