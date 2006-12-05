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
#ifndef _MBACLOCK_H_
#define _MBACLOCK_H_

#include <time.h>
class MBAclock {
  //long    i = 600000L;
  clock_t start, finish;
  double  duration;
  
public:
  MBAclock(){start = clock();}
  ~MBAclock(){};
  double getInterval() {
    finish = clock(); 
    duration = (double)(finish - start) / CLOCKS_PER_SEC;
    start = finish;
    return duration;
  }
};

#endif
