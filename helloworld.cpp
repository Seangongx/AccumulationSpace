#include <iostream>
#include <Eigen/Dense>
 
//using Eigen::MatrixXd;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
 
using namespace std;
 
int main()
{
        cout<<"*******************1D-object****************"<<endl;
        Vector4d v1;
        v1<< 1,2,3,4;
        cout<<"v1=\n"<<v1<<endl;
 
        VectorXd v2(3);
        v2<<1,2,3;
        cout<<"v2=\n"<<v2<<endl;
 
        Array4i v3;
        v3<<1,2,3,4;
        cout<<"v3=\n"<<v3<<endl;
 
        ArrayXf v4(3);
        v4<<1,2,3;
        cout<<"v4=\n"<<v4<<endl;
}

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