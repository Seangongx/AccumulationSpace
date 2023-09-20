#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"

#include <DGtal/shapes/Mesh.h>
#include <DGtal/io/readers/MeshReader.h>
#include <DGtal/io/writers/MeshWriter.h>
//#include <DGtal/io/viewers/Viewer3D.h>

#include <DGtal/images/ImageContainerBySTLVector.h>
#include "NormalAccumulator.h"

using namespace std;
using namespace DGtal;
using namespace DGtal::Z3i;

int main(int argc, char **argv)
{
    trace.info() <<"test\n";
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