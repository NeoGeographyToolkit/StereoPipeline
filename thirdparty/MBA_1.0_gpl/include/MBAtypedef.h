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
#ifndef _MBATYPEDEF_H
#define _MBATYPEDEF_H

/// For printing "debug" information to standard output
//#define MBA_DEBUG 1


#define MBA_UNDEFREAL 1572312345624422229996576879160.0

#include <GenMatrix.h>
#include <UCBtypedef.h>

#include <vector>

/** \interface dVec
* \brief typedef std::vector<double> dVec
*
*/
typedef std::vector<double> dVec;

/** \interface MBAbaseType
 * \brief enum MBAbaseType {MBA_ZERO, MBA_CONSTLS, MBA_CONSTVAL}
 *
 * This is the surface on which the spline surface is built over.
 * \see Details in MBA::setBaseType
 */
enum MBAbaseType {MBA_ZERO, MBA_CONSTLS, MBA_CONSTVAL}; // default is none

#endif
