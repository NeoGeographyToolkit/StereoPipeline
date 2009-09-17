//===========================================================================
// GoTools - SINTEF Geometry Tools version 1.0
//
// Multilevel B-spline Approximation module
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
#ifndef _UCB_UTILS_H_
#define _UCB_UTILS_H_

#include <UCBsplineSurface.h>
#include <vector>


#include <PointAccessUtils.h> // ??? Temporary here now so we dodn't need to include it in app.


/** \brief Misc. utilities, e.g., reading scattered data from file and
 *  printing surface to different plotting formats.
 *
 *  The functions are not documented in detail as many are self-explanatory.
 *  Consult the source code for details.
 */
namespace UCBspl {
  
  // Operations on UCBspl::SplineSurface, file access
  // ------------------------------------------------
  void printVRMLgrid(const char filename[], const UCBspl::SplineSurface& surf, int noU, int noV, double scale = 1.0);
  void printVTKgrid(const char filename[], const UCBspl::SplineSurface& surf, int noU, int noV, double scale = 1.0);
  void printVTKtriangleStrips(const char filename[], const UCBspl::SplineSurface& surf, int noU, int noV, double scale = 1.0);
  void printGNUgrid (const char filename[], const UCBspl::SplineSurface& surf, int noU, int noV);  
  void printIRAPgrid(const char filename[], const UCBspl::SplineSurface& surf, int noU, int noV);
  void printGLgrid(const char filename[], const UCBspl::SplineSurface& surf, int noU, int noV);
  void printGLgridBin(const char filename[], const UCBspl::SplineSurface& surf, int noU, int noV,
                      const std::vector<double>& X, const std::vector<double>& Y, const std::vector<double>& Z, // scattered data
                      double scale = 1.0);
  void saveSplineSurface(const char filename[], const UCBspl::SplineSurface& surf);
  void readSplineSurface(const char filename[], UCBspl::SplineSurface& surf);
  void saveSplineSurfaceBin(const char filename[], const UCBspl::SplineSurface& surf);
  void readSplineSurfaceBin(const char filename[], UCBspl::SplineSurface& surf);
  
}; // end namespace

#endif
