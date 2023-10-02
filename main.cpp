#include "DGtal/shapes/TriangulatedSurface.h"
#include "DGtal/shapes/Mesh.h"
#include "DGtal/shapes/MeshHelpers.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include <DGtal/io/readers/MeshReader.h>
#include <DGtal/io/writers/MeshWriter.h>
#include "NormalAccumulator.h"

#include "CLI11.hpp"

using namespace std;
using namespace DGtal;
using namespace DGtal::Z3i;

int main(int argc, char **argv)
{
  string inputFile;
  double inputRadius{5.0};
  string inputTypeStat{"min"};

  // parse command line using CLI ----------------------------------------------
  CLI::App app;
  app.description("Compute mesh accumulation from a mesh and compute the radius (median value) "
                  "of all faces participating to the accumulation");
  app.add_option("-i,--input,1", inputFile, "input mesh.")
      ->required()
      ->check(CLI::ExistingFile);

  app.add_option("--radius,-r", inputRadius, "radius of accumulation analysis.", true);
  app.add_option("--radiusEstimator,-e", inputTypeStat, "use: {min (default), max, mean, median} to estimate the radius", true)
      ->check(CLI::IsMember({"max", "min", "mean", "median"}));
  app.get_formatter()->column_width(40);
  CLI11_PARSE(app, argc, argv);
  // END parse command line using CLI ----------------------------------------------

  // 1) Reading input mesh
  Mesh<Z3i::RealPoint> tempMesh(true);
  tempMesh << inputFile;

  NormalAccumulator na(inputRadius, inputTypeStat);

  return 0;
}

// RealVector a(1, 1, 1);
// RealVector b(2, 4, 6);
// RealVector c = a.crossProduct(b);
// cout << a << endl;
// cout << b << endl;
// cout << c << endl;

// typedef TriangulatedSurface<RealPoint> TriMesh;
// typedef Mesh<RealPoint> ViewMesh;

// // Creates two triangles glued together.
// TriMesh tmesh;
// tmesh.addVertex(RealPoint(0, 0, 0));
// tmesh.addVertex(RealPoint(1, 0, 0));
// tmesh.addVertex(RealPoint(0, 1, 0));
// tmesh.addVertex(RealPoint(1, 1, 1));
// tmesh.addTriangle(0, 1, 2);
// tmesh.addTriangle(2, 1, 3);
// tmesh.build();

// // Convert it to a mesh
// ViewMesh mesh;
// MeshHelpers::triangulatedSurface2Mesh(tmesh, mesh);

// // View it
// QApplication application(argc, argv);
// Viewer3D<> viewer;
// viewer.show();
// viewer.setLineColor(Color(150, 0, 0, 254));
// viewer << mesh;
// viewer << Viewer3D<>::updateDisplay;
// application.exec();

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