/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file
 * @ingroup visualisation
 * @author Bertrand Kerautret (\c bertrand.kerautret@univ-lyon2.fr )
 *         Xun Gong (\c sean.gong814@gmail.com)
 *
 * @date 2024/06/18
 *
 * Extended functionalities for the source file of PolyMeshEdit tool
 *
 * This file is part of the DGtal library/DGtalTools-contrib Project.
 */

///////////////////////////////////////////////////////////////////////////////
#include <sys/stat.h>
#include <string>
#include "CLI11.hpp"
#include "PolyscopeEnvironment.h"
#include "PolyscopeEnvironment.ipp"
///////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace DGtal;
///////////////////////////////////////////////////////////////////////////////

/**
 @page polyAccEdit polyAccEdit

 @brief  polyAccEdit tools to display an accumulation voxel set (Using point
 cloud for this experiement) calculated from a mesh. Click an accumulation voxel
 inside the mesh or a triangle face on the mesh can visualize the associated
 faces or voting accumulations on the fly.

 @b Usage:   polyAccEdit [OPTIONS] 1 2


 @b Allowed @b options @b are :

 @code

 Positionals:
   1 TEXT:FILE REQUIRED                  an input mesh file in .obj or .off
 format. 2 TEXT:FILE EQUIRED                an input accumulation file in .dat
 format.

 Options:
   -h,--help                             Print this help message and exit
   -i,--input TEXT:FILE REQUIRED         an input mesh file in .obj or .off
 format.
   -l,--logLevel INT:INT                 0: DEBUG, 1: INFO, 2: WARNING, 3: ERROR
 @endcode

 @b Example:

 @code
    polyAccEdit /Samples/xxx.obj /Samples/xxx.dat
 @endcode

 @image html respolyAccEdit.png "Example of result. "

 @see
 @ref polyAccEdit.cpp

 */

int main(int argc, char** argv) {

  std::string defaultMeshFile{""};
  std::string defaultAccFile{""};
  std::string defaultLogFile{"TestLog.txt"};
  AccumulationSpace::LogLevel logLevel = AccumulationSpace::LogLevel::INFO;

  // parse command line using CLI ----------------------------------------------
  CLI::App app;
  app.description(
      "polyAccEdit tools to display an accumulation voxel set (Using point "
      "cloud for this experiement) calculated from a mesh. Click an "
      "accumulation voxel inside the mesh or a triangle face on the mesh can "
      "visualize the associated faces or voting accumulations on the fly."
      "polyAccEdit /Samples/xxx.obj /Samples/xxx.dat \n");
  app.add_option("-i,--input,1", defaultMeshFile,
                 "an input mesh file in .obj or .off format.")
      ->required()
      ->check(CLI::ExistingFile);
  app.add_option("-a,--inputAcc,2", defaultAccFile,
                 "an input accumulation file in .dat format.", true);
  app.add_option("-l,--logLevel,3", logLevel,
                 "0: DEBUG, 1: INFO, 2: WARNING, 3: ERROR", true);
  app.add_option("-m,--logName,4", defaultLogFile, "the log file name", true);

  app.get_formatter()->column_width(40);
  CLI11_PARSE(app, argc, argv);

  // Initialize log file
  auto globalLogFileStream = std::make_shared<std::fstream>(
      defaultLogFile, std::ios::out | std::ios::trunc);
  AccumulationSpace::AccumulationLog defaultLog(defaultLogFile, logLevel,
                                                globalLogFileStream);

  // Initialize polyscope environment
  std::shared_ptr<AccumulationSpace::AccumulationLog> defaultLogPtr =
      std::make_shared<AccumulationSpace::AccumulationLog>(defaultLog);
  PolyscopeEnvironment::Manager psManager{defaultMeshFile, defaultAccFile,
                                          defaultLogPtr};

  polyscope::shutdown();
  return 0;
}
