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
#include <cstddef>
#include <string>
#include <sys/stat.h>
///////////////////////////////////////////////////////////////////////////////
#include "DGtal/base/Common.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/MeshReader.h"
///////////////////////////////////////////////////////////////////////////////
#include "glm/fwd.hpp"
#include "imgui.h"
#include "polyscope/pick.h"
#include "polyscope/point_cloud.h"
#include "polyscope/point_cloud.ipp"
#include "polyscope/polyscope.h"
#include "polyscope/render/color_maps.h"
#include "polyscope/render/engine.h"
#include "polyscope/surface_mesh.h"
///////////////////////////////////////////////////////////////////////////////
#include "CLI11.hpp"
#include "PolyscopeEnvironment.h"
#include "PolyscopeEnvironment.ipp"
#include "Timer.h"
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

  // parse command line using CLI ----------------------------------------------
  CLI::App app;
  app.description("polyAccEdit tools to display an accumulation voxel set (Using point "
                  "cloud for this experiement) calculated from a mesh. Click an "
                  "accumulation voxel inside the mesh or a triangle face on the mesh can "
                  "visualize the associated faces or voting accumulations on the fly."
                  "polyAccEdit /Samples/xxx.obj /Samples/xxx.dat \n");
  app.add_option("-i,--input,1", defaultMeshFile, "an input mesh file in .obj or .off format.")
      ->required()
      ->check(CLI::ExistingFile);
  app.add_option("-a,--inputAcc,2", defaultAccFile, "an input accumulation file in .dat format.", true);

  app.get_formatter()->column_width(40);
  CLI11_PARSE(app, argc, argv);

  // Build environment
  PolyscopeEnvironment::Manager psManager{defaultMeshFile, defaultAccFile};

  return 0;
}
