#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"

#include <DGtal/shapes/Mesh.h>
#include <DGtal/io/readers/MeshReader.h>
#include <DGtal/io/writers/MeshWriter.h>
#include <DGtal/io/viewers/Viewer3D.h>

#include <DGtal/images/ImageContainerBySTLVector.h>
#include "NormalAccumulator.h"

using namespace std;
using namespace DGtal;
using namespace DGtal::Z3i;

int main(int argc, char **argv)
{
  // 
  QApplication application(argc,argv);
  Viewer3D<> viewer;
  viewer.show();
 
 
  // A mesh is constructed and faces are added from the vertex set.
  Mesh<Point> aMesh(true);
 
  aMesh.addVertex(Point(0,0,0));
  aMesh.addVertex(Point(1,0,0));
  aMesh.addVertex(Point(1,1,0));
 
  aMesh.addVertex(Point(0,0,1));
  aMesh.addVertex(Point(1,0,1));
  aMesh.addVertex(Point(1,1,1));
  aMesh.addVertex(Point(0,1,1));
 
  aMesh.addVertex(Point(0,1,0));
  aMesh.addVertex(Point(0,2,0));
  aMesh.addVertex(Point(0,3,1));
  aMesh.addVertex(Point(0,2,2));
  aMesh.addVertex(Point(0,1,2));
  aMesh.addVertex(Point(0,0,1));
  aMesh.addTriangularFace(0, 1, 2, Color(150,0,150,104));
  aMesh.addQuadFace(6,5,4,3, Color::Blue);
 
  Mesh<Point>::MeshFace listIndex;
  listIndex.push_back(7);
  listIndex.push_back(8);
  listIndex.push_back(9);
  listIndex.push_back(10);
  listIndex.push_back(11);
  listIndex.push_back(12);
  aMesh.addFace(listIndex, Color(150,150,0,54));
 
  viewer.setLineColor(Color(150,0,0,254));
  viewer << aMesh;
  viewer << Viewer3D<>::updateDisplay;
  bool res = application.exec();
  FATAL_ERROR(res);
  return 0;
}

// #include <iostream>
// #include <Eigen/Dense>

// //using Eigen::MatrixXd;
// using namespace Eigen;
// using namespace Eigen::internal;
// using namespace Eigen::Architecture;

// using namespace std;

// int main()
// {
//         cout<<"*******************1D-object****************"<<endl;
//         Vector4d v1;
//         v1<< 1,2,3,4;
//         cout<<"v1=\n"<<v1<<endl;

//         VectorXd v2(3);
//         v2<<1,2,3;
//         cout<<"v2=\n"<<v2<<endl;

//         Array4i v3;
//         v3<<1,2,3,4;
//         cout<<"v3=\n"<<v3<<endl;

//         ArrayXf v4(3);
//         v4<<1,2,3;

//         cout<<"v4=\n"<<v4<<endl;
// }

// #include <boost/lambda/lambda.hpp>
// #include <iostream>
// #include <iterator>
// #include <algorithm>

// int main()
// {
//     using namespace boost::lambda;
//     typedef std::istream_iterator<int> in;

//     std::for_each(
//         in(std::cin), in(), std::cout << (_1 * 3) << " " );
// }

// #include <iostream>
// #include <vector>
// #include <string>

// using namespace std;

// int main()
// {
//     vector<string> msg {"Hello", "C++", "World", "from", "VS Code", "and the C++ extension again!"};

//     for (const string& word : msg)
//     {
//         cout << word << " ";
//     }
//     getchar();

//     cout << endl;
// }