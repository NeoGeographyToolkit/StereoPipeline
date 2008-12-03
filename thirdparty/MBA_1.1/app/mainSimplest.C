//===========================================================================
// SINTEF Multilevel B-spline Approximation library - version 1.1
//
// Copyright (C) 2000-2005 SINTEF ICT, Applied Mathematics, Norway.
//
// This program is free software; you can redistribute it and/or          
// modify it under the terms of the GNU General Public License            
// as published by the Free Software Foundation version 2 of the License. 
//
// This program is distributed in the hope that it will be useful,        
// but WITHOUT ANY WARRANTY; without even the implied warranty of         
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          
// GNU General Public License for more details.                           
//
// You should have received a copy of the GNU General Public License      
// along with this program; if not, write to the Free Software            
// Foundation, Inc.,                                                      
// 59 Temple Place - Suite 330,                                           
// Boston, MA  02111-1307, USA.                                           
//
// Contact information: e-mail: tor.dokken@sintef.no                      
// SINTEF ICT, Department of Applied Mathematics,                         
// P.O. Box 124 Blindern,                                                 
// 0314 Oslo, Norway.                                                     
//
// Other licenses are also available for this software, notably licenses
// for:
// - Building commercial software.                                        
// - Building software whose source code you wish to keep private.        
//===========================================================================
//! \page mainsimplest A complete but simple main program
/// \include mainSimplest.C
#include <MBA.h>
#include <UCButils.h>
#include <PointAccessUtils.h>


int main() {

  // Read scattered data from the file Data/rygg1.dat.
  // The format is assumed to be:
  // x y z
  // x y z
  // x y z
  // etc.

  typedef std::vector<double> dVec;
  boost::shared_ptr<dVec> x_arr(new std::vector<double>);
  boost::shared_ptr<dVec> y_arr(new std::vector<double>);
  boost::shared_ptr<dVec> z_arr(new std::vector<double>);
  UCBspl::readScatteredData("Data/rygg1.dat", *x_arr, *y_arr, *z_arr);

  MBA mba(x_arr, y_arr, z_arr);

  // Create spline surface.
  mba.MBAalg(1,1,7);

  UCBspl::SplineSurface surf = mba.getSplineSurface();

  // Find height and normal vector of surface in (x,y).
  double x = 5.0, y = 5.0;
  double z = surf.f(x,y);         
  double nx, ny, nz;
  surf.normalVector(x, y, nx, ny, nz);
  std::cout << "z-value in (5,5) = " << z << std::endl;
  std::cout << "Normal in (5,5) = (" << nx << "," << ny << "," << nz << ")"
	    << std::endl;

  // Sample surface and print to VRML file.
  UCBspl::printVRMLgrid("qwe.wrl", surf, 50, 50);  

  return 0;
}
