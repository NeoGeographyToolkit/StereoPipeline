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
#ifndef _UCB_TYPEDEF_H_
#define _UCB_TYPEDEF_H_


/** \interface UCBspl_real
 *  \brief \#define UCBspl_real float; //Real type for large data objects (matrices)
 *
 *  Defines the real type for for large data objects such as vector and matrices.
 *  (Should be float or double) 
 */
#ifndef UCBspl_real
#define UCBspl_real float
#endif

template <class Type>
class GenMatrix; //<Type>;
/** \interface GenMatrixType 
 * \brief typedef GenMatrix<UCBspl_real> GenMatrixType;
 */
typedef GenMatrix<UCBspl_real> GenMatrixType;


#endif
