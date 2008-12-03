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
#ifndef _FILTERRESTRICT2D_H
#define _FILTERRESTRICT2D_H

#include "UCBtypedef.h"

void filterRestrict2D(int m,                // width and height of filter (2m+1)
		      const UCBspl_real* F, // complete 2D filter F(i,j) = F[j * (2m+1) + i]
		      bool left,            // true = left boundary (index i)
		      bool lower,           // true = lower boundary, (index j)
		      int k,                // number of depassing coefficients in i direction
		      int l,                // number of depassing coefficients in j direction
		      UCBspl_real* result); // pointer to memory area where the result is written
		      
// NB: k and l must both be in [0, m].  If both are equal to 0, then the unrestricted operator
// is returned.  In general, the result operator will have the resolution (2m+1-k, 2m+1-l).
// The original filter and the result are both stored row-wise (i has lowest stride).

#endif // _FILTERRESTRICT2D_H

