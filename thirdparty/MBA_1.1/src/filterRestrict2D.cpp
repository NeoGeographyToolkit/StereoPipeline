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

#include <stdexcept>
#include <vector>
#include "filterRestrict2D.h"

using namespace std;

namespace {
    void argument_check(int m, const UCBspl_real* F, int k, int l, const UCBspl_real* result);
}; 


void filterRestrict2D(int m,                
		      const UCBspl_real* F, 
		      bool left,            
		      bool lower,           
		      int k,                
		      int l,                
		      UCBspl_real* result)  
{
    argument_check(m, F, k, l, result);

    int rlen = 2 * m + 1;
    std::vector<UCBspl_real> Ftemp(F, F + rlen * rlen);

    if (left) {
	for (int j = 0; j < rlen; ++j) {
	    for (int i = 1; i <= k; ++i) {
		UCBspl_real temp = Ftemp[j * rlen + k - i];
		Ftemp[j * rlen + k] += (i+1) * temp;
		Ftemp[j * rlen + k + 1] += (-i) * temp;
	    }
	} 
    } else {
	for (int j = 0; j < rlen; ++j) {
	    for (int i = 1; i <= k; ++i) {
		UCBspl_real temp = Ftemp[j * rlen + (rlen - 1) - k + i];
		Ftemp[j * rlen + (rlen - 1) - k] += (i+1) * temp;
		Ftemp[j * rlen + (rlen - 1) - k - 1] += (-i) * temp;
	    }
	}
    }
    if (lower) {
	for (int i = 0; i < rlen; ++i) {
	    for (int j = 1; j <= l; ++j) {
		UCBspl_real temp = Ftemp[(l - j) * rlen + i];
		Ftemp[l * rlen + i] += (j+1) * temp;
		Ftemp[(l+1) * rlen + i] += (-j) * temp;
	    }
	}
    } else {
	for (int i = 0; i < rlen; ++i) {
	    for (int j = 1; j <= l; ++j) {
		UCBspl_real temp = Ftemp[(rlen - 1 - l + j) * rlen + i];
		Ftemp[(rlen - 1 - l) * rlen + i] += (j+1) * temp;
		Ftemp[(rlen - 1 - 1 - l) * rlen + i] += (-j) * temp;
	    }
	}
    }


    int lower_bound_i = 0;
    int upper_bound_i = rlen;
    int lower_bound_j = 0;
    int upper_bound_j = rlen;
    if (left) {
	lower_bound_i += k;
    } else {
	upper_bound_i -= k;
    }
    if (lower) {
	lower_bound_j += l;
    } else {
	upper_bound_j -= l;
    }
    
    int pos = 0;
    for (int j = lower_bound_j; j < upper_bound_j; ++j) {
	for (int i = lower_bound_i; i < upper_bound_i; ++i) {
	    result[pos++] = Ftemp[j * rlen + i];
	}
    }
}

namespace {

void argument_check(int m, const UCBspl_real* F, int k, int l, const UCBspl_real* result)
{
    if (m < 1) {
	throw runtime_error("Invalid size of filter.");
    }
    if (!F || !result) {
	throw runtime_error("Null pointers detected.");
    }
    if (k < 0 || k > m) {
	throw runtime_error("Invalid value for k (number of steps outside boundary).  "
			    "Should be between 0 and m.");
    }
    if (k < 0 || k > m) {
	throw runtime_error("Invalid value for l (number of steps outside boundary).  "
			    "Should be between 0 and m.");
    }
}

}; 
 
