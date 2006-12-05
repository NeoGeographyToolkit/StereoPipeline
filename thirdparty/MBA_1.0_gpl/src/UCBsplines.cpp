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
#include <UCBsplines.h>

#ifdef DEBUG_UCBspl
#include <iostream>
#endif

void UCBspl::refineCoeffsC2(const GenMatrix<UCBspl_real>& PSI, GenMatrix<UCBspl_real>& PSIprime) {
  
  int mm = PSI.noX()-3; 
  int nn = PSI.noY()-3; 
  
  PSIprime.resize(2*mm+3, 2*nn+3);
  
  int i,j;
  
  for (j = 0; j <= nn; j++) {
    PSIprime(-1,2*j-1) = phi_2iPlus1_2jPlus1(PSI,-1,j-1);   
    PSIprime(-1,2*j)   = phi_2iPlus1_2j(PSI,-1,j);          
    
    for (i = 0; i <= mm; i++) {
      PSIprime(2*i,2*j)     = phi_2i_2j(PSI,i,j);           
      PSIprime(2*i,2*j+1)   = phi_2i_2jPluss1(PSI,i,j);     
      PSIprime(2*i+1,2*j)   = phi_2iPlus1_2j(PSI,i,j);      
      PSIprime(2*i+1,2*j+1) = phi_2iPlus1_2jPlus1(PSI,i,j); 
    }
  }
  
  PSIprime(-1,2*nn+1) = phi_2iPlus1_2jPlus1(PSI,-1,nn);     
  
  for (i = 0; i <= mm; i++) {
    PSIprime(2*i,-1)   = phi_2i_2jPluss1(PSI,i,-1);      
    PSIprime(2*i+1,-1) = phi_2iPlus1_2jPlus1(PSI,i,-1);  
  }
}


void UCBspl::refineCoeffsC1(const GenMatrix<UCBspl_real>& PSI, GenMatrix<UCBspl_real>& PSIprime) {
  
  int mm = (PSI.noX()-2)/2; 
  int nn = (PSI.noY()-2)/2; 
  
  PSIprime.resize(4*mm+2, 4*nn+2);
  
  int i,j;
  for (j = 0; j < nn; j++) {
    
    int jj = 2*j;
    for (i = 0; i < mm; i++) {
      int ii = 2*i;
      
      PSIprime(4*i-1,4*j-1) = phi_4iMin1_4jMin1(PSI,ii,jj);
      PSIprime(4*i-1,4*j) = phi_4iMin1_4j(PSI,ii,jj);
      PSIprime(4*i,4*j-1) = phi_4i_4jMin1(PSI,ii,jj);
      PSIprime(4*i,4*j) = phi_4i_4j(PSI,ii,jj);
      
      PSIprime(4*i-1,4*j+1) = phi_4iMin1_4jPlus1(PSI,ii,jj);
      PSIprime(4*i-1,4*j+2) = phi_4iMin1_4jPlus2(PSI,ii,jj);
      PSIprime(4*i,4*j+1) = phi_4i_4jPlus1(PSI,ii,jj);
      PSIprime(4*i,4*j+2) = phi_4i_4jPlus2(PSI,ii,jj);
      
      PSIprime(4*i+1,4*j-1) = phi_4iPlus1_4jMin1(PSI,ii,jj);
      PSIprime(4*i+1,4*j) = phi_4iPlus1_4j(PSI,ii,jj);
      PSIprime(4*i+2,4*j-1) = phi_4iPlus2_4jMin1(PSI,ii,jj);
      PSIprime(4*i+2,4*j) = phi_4iPlus2_4j(PSI,ii,jj);
      
      PSIprime(4*i+1,4*j+1) = phi_4iPlus1_4jPlus1(PSI,ii,jj);
      PSIprime(4*i+1,4*j+2) = phi_4iPlus1_4jPlus2(PSI,ii,jj);
      PSIprime(4*i+2,4*j+1) = phi_4iPlus2_4jPlus1(PSI,ii,jj);
      PSIprime(4*i+2,4*j+2) = phi_4iPlus2_4jPlus2(PSI,ii,jj);
    }
    
    int ii = 2*mm;
    PSIprime(4*mm-1,4*j-1) = phi_4iMin1_4jMin1(PSI,ii,jj);
    PSIprime(4*mm-1,4*j) = phi_4iMin1_4j(PSI,ii,jj);
    PSIprime(4*mm,4*j-1) = phi_4i_4jMin1(PSI,ii,jj);
    PSIprime(4*mm,4*j) = phi_4i_4j(PSI,ii,jj);
    
    PSIprime(4*mm-1,4*j+1) = phi_4iMin1_4jPlus1(PSI,ii,jj);
    PSIprime(4*mm-1,4*j+2) = phi_4iMin1_4jPlus2(PSI,ii,jj);
    PSIprime(4*mm,4*j+1) = phi_4i_4jPlus1(PSI,ii,jj);
    PSIprime(4*mm,4*j+2) = phi_4i_4jPlus2(PSI,ii,jj);
  }
  
  int jj = 2*nn;
  for (i = 0; i < mm; i++) {
    int ii = 2*i;
    PSIprime(4*i-1,4*nn-1) = phi_4iMin1_4jMin1(PSI,ii,jj);
    PSIprime(4*i-1,4*nn)   = phi_4iMin1_4j(PSI,ii,jj);
    PSIprime(4*i,4*nn-1)   = phi_4i_4jMin1(PSI,ii,jj);
    PSIprime(4*i,4*nn)     = phi_4i_4j(PSI,ii,jj);
    
    PSIprime(4*i+1,4*nn-1) = phi_4iPlus1_4jMin1(PSI,ii,jj);
    PSIprime(4*i+1,4*nn)   = phi_4iPlus1_4j(PSI,ii,jj);
    PSIprime(4*i+2,4*nn-1) = phi_4iPlus2_4jMin1(PSI,ii,jj);
    PSIprime(4*i+2,4*nn)   = phi_4iPlus2_4j(PSI,ii,jj);
  }
  
  int ii = 2*mm;
  PSIprime(4*mm-1,4*nn-1) = phi_4iMin1_4jMin1(PSI,ii,jj);
  PSIprime(4*mm-1,4*nn)   = phi_4iMin1_4j(PSI,ii,jj);
  PSIprime(4*mm,4*nn-1)   = phi_4i_4jMin1(PSI,ii,jj);
  PSIprime(4*mm,4*nn)     = phi_4i_4j(PSI,ii,jj);
}


bool UCBspl::restrictCoeffsC2(const GenMatrix<UCBspl_real>& rr, GenMatrix<UCBspl_real>& r) {
  
  int old_noX = rr.noX();
  int old_noY = rr.noY();
  
  
  if ((old_noX-3)%2 != 0 || (old_noY-3)%2 != 0) {
#ifdef DEBUG_UCBspl
    cout << "ERROR, invalid grid for projection; see comments in the code above: " << old_noX << " " << old_noY << endl;
    exit(-1);
#endif
    return false;
  }
  
  int noX = (old_noX-3)/2 + 3;
  int noY = (old_noY-3)/2 + 3;
  r.resize(noX, noY);
  
  double denominatorFull   = 256.0; 
  double denominatorCorner =  25.0; 
  double denominatorOp2    = 225.0;
  double denominatorOp4    =  80.0;
  double denominatorOp5    = 240.0;
  double denominatorOp6    =  75.0;
  
  int kk,ll,kk_new,ll_new;
  
  for (kk_new = 1; kk_new <= noX-4; kk_new++) {
    for (ll_new = 1; ll_new <= noY-4; ll_new++) {
      kk = 2*kk_new; 
      ll = 2*ll_new;      
      double val = rr(kk-2,ll-2) + rr(kk-2,ll+2) + rr(kk+2,ll-2) + rr(kk+2,ll+2) 
        + 16.0 * (rr(kk-1,ll-1) + rr(kk-1,ll+1) + rr(kk+1,ll-1) + rr(kk+1,ll+1)) 
        +  6.0 * (rr(kk,ll-2) + rr(kk,ll+2) + rr(kk-2,ll) + rr(kk+2,ll)) 
        + 24.0 * (rr(kk,ll-1) + rr(kk,ll+1) + rr(kk-1,ll) + rr(kk+1,ll)) 
        +  4.0 * (rr(kk-2,ll-1) + rr(kk-1,ll-2) + rr(kk+1,ll-2) + rr(kk+2,ll-1)) 
        +  4.0 * (rr(kk-2,ll+1) + rr(kk-1,ll+2) + rr(kk+1,ll+2) + rr(kk+2,ll+1)) 
        + 36.0 * rr(kk,ll);
      r(kk_new,ll_new) = val/denominatorFull;
    }
  }
  
  
  for (kk_new = 1; kk_new <= noX-4; kk_new++) {
    
    double val;
    kk = 2*kk_new;
    
    ll_new = noY-2;
    ll = 2*ll_new-1;
    val = rr(kk-2,ll-1) + rr(kk+2,ll-1) 
      + 16.0 * (rr(kk-1,ll) + rr(kk+1,ll)) 
      +  6.0 *  rr(kk,ll-1) 
      + 24.0 *  rr(kk,ll)   
      +  4.0 * (rr(kk-2,ll) + rr(kk+2,ll)) 
      +  4.0 * (rr(kk-1,ll-1) + rr(kk+1,ll-1)); 
    r(kk_new,ll_new) = val/denominatorOp4;
    
    ll_new = noY-3;
    ll = 2*ll_new;
    val = rr(kk-2,ll-2) + rr(kk+2,ll-2) 
      + 16.0 * (rr(kk-1,ll-1) + rr(kk-1,ll+1) + rr(kk+1,ll-1) + rr(kk+1,ll+1)) 
      +  6.0 * (rr(kk,ll-2) + rr(kk-2,ll) + rr(kk+2,ll)) 
      + 24.0 * (rr(kk,ll-1) + rr(kk,ll+1) + rr(kk-1,ll) + rr(kk+1,ll)) 
      +  4.0 * (rr(kk-2,ll-1) + rr(kk-1,ll-2) + rr(kk+1,ll-2) + rr(kk+2,ll-1)) 
      +  4.0 * (rr(kk-2,ll+1) + rr(kk+2,ll+1)) 
      + 36.0 * rr(kk,ll);
    r(kk_new,ll_new) = val/denominatorOp5;
    
    ll_new = ll = -1;
    val = rr(kk-2,ll+1) + rr(kk+2,ll+1) 
      + 16.0 * (rr(kk-1,ll) + rr(kk+1,ll)) 
      +  6.0 *  rr(kk,ll+1) 
      + 24.0 *  rr(kk,ll)   
      +  4.0 * (rr(kk-2,ll) + rr(kk+2,ll)) 
      +  4.0 * (rr(kk-1,ll+1) + rr(kk+1,ll+1)); 
    r(kk_new,ll_new) = val/denominatorOp4;
    
    ll_new = ll = 0;
    val = rr(kk-2,ll+2) + rr(kk+2,ll+2) 
      + 16.0 * (rr(kk-1,ll-1) + rr(kk-1,ll+1) + rr(kk+1,ll-1) + rr(kk+1,ll+1)) 
      +  6.0 * (rr(kk,ll+2) + rr(kk-2,ll) + rr(kk+2,ll)) 
      + 24.0 * (rr(kk,ll-1) + rr(kk,ll+1) + rr(kk-1,ll) + rr(kk+1,ll)) 
      +  4.0 * (rr(kk-2,ll+1) + rr(kk-1,ll+2) + rr(kk+1,ll+2) + rr(kk+2,ll+1)) 
      +  4.0 * (rr(kk-2,ll-1) + rr(kk+2,ll-1)) 
      + 36.0 * rr(kk,ll);
    r(kk_new,ll_new) = val/denominatorOp5;
  } 
  
  
  
  for (ll_new = 1; ll_new <= noY-4; ll_new++) {
    
    double val;
    ll = 2*ll_new;
    
    kk_new = noX-2;
    kk = 2*kk_new-1;
    val = rr(kk-1,ll+2) + rr(kk-1,ll-2) 
      + 16.0 * (rr(kk,ll+1) + rr(kk,ll-1)) 
      +  6.0 *  rr(kk-1,ll) 
      + 24.0 *  rr(kk,ll)   
      +  4.0 * (rr(kk,ll+2) + rr(kk,ll-2)) 
      +  4.0 * (rr(kk-1,ll+1) + rr(kk-1,ll-1)); 
    r(kk_new,ll_new) = val/denominatorOp4;
    
    kk_new = noX-3;
    kk = 2*kk_new;
    val = rr(kk-2,ll-2) + rr(kk-2,ll+2) 
      + 16.0 * (rr(kk-1,ll-1) + rr(kk-1,ll+1) + rr(kk+1,ll-1) + rr(kk+1,ll+1)) 
      +  6.0 * (rr(kk,ll+2) + rr(kk,ll-2) + rr(kk-2,ll)) 
      + 24.0 * (rr(kk,ll-1) + rr(kk,ll+1) + rr(kk-1,ll) + rr(kk+1,ll)) 
      +  4.0 * (rr(kk-1,ll+2) + rr(kk-1,ll-2) + rr(kk-2,ll+1) + rr(kk-2,ll-1)) 
      +  4.0 * (rr(kk+1,ll+2) + rr(kk+1,ll-2)) 
      + 36.0 * rr(kk,ll);
    r(kk_new,ll_new) = val/denominatorOp5;
    
    kk_new = kk = -1;
    val = rr(kk+1,ll+2) + rr(kk+1,ll-2) 
      + 16.0 * (rr(kk,ll+1) + rr(kk,ll-1)) 
      +  6.0 *  rr(kk+1,ll) 
      + 24.0 *  rr(kk,ll)   
      +  4.0 * (rr(kk,ll+2) + rr(kk,ll-2)) 
      +  4.0 * (rr(kk+1,ll+1) + rr(kk+1,ll-1)); 
    r(kk_new,ll_new) = val/denominatorOp4;
    
    kk_new = kk = 0;
    val = rr(kk+2,ll+2) + rr(kk+2,ll-2) 
      + 16.0 * (rr(kk-1,ll-1) + rr(kk-1,ll+1) + rr(kk+1,ll-1) + rr(kk+1,ll+1)) 
      +  6.0 * (rr(kk,ll+2) + rr(kk,ll-2) + rr(kk+2,ll)) 
      + 24.0 * (rr(kk,ll-1) + rr(kk,ll+1) + rr(kk-1,ll) + rr(kk+1,ll)) 
      +  4.0 * (rr(kk+1,ll+2) + rr(kk+1,ll-2) + rr(kk+2,ll+1) + rr(kk+2,ll-1)) 
      +  4.0 * (rr(kk-1,ll+2) + rr(kk-1,ll-2)) 
      + 36.0 * rr(kk,ll);
    r(kk_new,ll_new) = val/denominatorOp5;
  }
  
  
  
  
  double val = 16.0*rr(-1,-1) + 4.0*(rr(-1,0)+rr(0,-1)) + rr(0,0);
  r(-1,-1) = val/denominatorCorner;
  
  val = 16.0*rr(old_noX-2,old_noY-2) + 4.0*(rr(old_noX-3,old_noY-2)+rr(old_noX-2,old_noY-3)) 
    + rr(old_noX-3,old_noY-3);
  r(noX-2,noY-2) = val/denominatorCorner;
  
  val = 16.0*rr(old_noX-2,-1) + 4.0*(rr(old_noX-2,0)+rr(old_noX-3,-1)) + rr(old_noX-3,0);
  r(noX-2,-1) = val/denominatorCorner;
  
  val = 16.0*rr(-1,old_noY-2) + 4.0*(rr(-1,old_noY-3)+rr(0,old_noY-2)) + rr(0,old_noY-3);
  r(-1,noY-2) = val/denominatorCorner;
  
  
  val = 36.0 * rr(0,old_noY-3)
    +16.0 * (rr(-1,old_noY-2) + rr(-1,old_noY-4) + rr(1,old_noY-2) + rr(1,old_noY-4))
    +24.0 * (rr(-1,old_noY-3) + rr(1,old_noY-3) + rr(0,old_noY-2) + rr(0,old_noY-4))
    + 6.0 * (rr(0,old_noY-5)  + rr(2,old_noY-3))
    + 4.0 * (rr(-1,old_noY-5) + rr(1,old_noY-5) + rr(2,old_noY-4) + rr(2,old_noY-2));
  r(0,noY-3) = val/denominatorOp2;
  
  val = 36.0 * rr(0,0)
    +16.0 * (rr(-1,-1) + rr(-1,1) + rr(1,-1) + rr(1,1))
    +24.0 * (rr(-1,0) + rr(1,0) + rr(0,-1) + rr(0,1))
    + 6.0 * (rr(0,2)  + rr(2,0))
    + 4.0 * (rr(2,-1) + rr(2,1) + rr(1,2) + rr(-1,2));
  r(0,0) = val/denominatorOp2;
  
  val = 36.0 * rr(old_noX-3,old_noY-3)
    +16.0 * (rr(old_noX-4,old_noY-4) + rr(old_noX-4,old_noY-2) + rr(old_noX-2,old_noY-4) + rr(old_noX-2,old_noY-2))
    +24.0 * (rr(old_noX-4,old_noY-3) + rr(old_noX-3,old_noY-4) + rr(old_noX-2,old_noY-3) + rr(old_noX-3,old_noY-2))
    + 6.0 * (rr(old_noX-5,old_noY-3)  + rr(old_noX-3,old_noY-5))
    + 4.0 * (rr(old_noX-5,old_noY-2) + rr(old_noX-5,old_noY-4) + rr(old_noX-4,old_noY-5) + rr(old_noX-2,old_noY-5));
  r(noX-3,noY-3) = val/denominatorOp2;
  
  val = 36.0 * rr(old_noX-3,0)
    +16.0 * (rr(old_noX-4,-1) + rr(old_noX-2,-1) + rr(old_noX-2,1) + rr(old_noX-4,1))
    +24.0 * (rr(old_noX-4,0) + rr(old_noX-3,-1) + rr(old_noX-2,0) + rr(old_noX-3,1))
    + 6.0 * (rr(old_noX-5,0)  + rr(old_noX-3,2))
    + 4.0 * (rr(old_noX-5,-1) + rr(old_noX-5,1) + rr(old_noX-4,2) + rr(old_noX-2,2));
  r(noX-3,0) = val/denominatorOp2;
  
  
  val = 24.0 * rr(0,old_noY-2)
    +16.0 * (rr(-1,old_noY-2) + rr(1,old_noY-2))
    + 4.0 * (rr(-1,old_noY-3) + rr(1,old_noY-3) + rr(2,old_noY-2))
    + 6.0 *  rr(0,old_noY-3)
    +       rr(2,old_noY-3);
  r(0,noY-2) = val/denominatorOp6;
  
  val = 24.0 * rr(-1,old_noY-3)
    +16.0 * (rr(-1,old_noY-2) + rr(-1,old_noY-4))
    + 4.0 * (rr(-1,old_noY-5) + rr(0,old_noY-4) + rr(0,old_noY-2))
    + 6.0 *  rr(0,old_noY-3)
    +       rr(0,old_noY-5);
  r(-1,noY-3) = val/denominatorOp6;
  
  val = 24.0 * rr(-1,0)
    +16.0 * (rr(-1,-1) + rr(-1,1))
    + 4.0 * (rr(0,-1) + rr(0,1) + rr(-1,2))
    + 6.0 *  rr(0,0)
    +       rr(0,2);
  r(-1,0) = val/denominatorOp6;
  
  val = 24.0 * rr(0,-1)
    +16.0 * (rr(-1,-1) + rr(1,-1))
    + 4.0 * (rr(-1,0) + rr(1,0) + rr(2,-1))
    + 6.0 *  rr(0,0)
    +       rr(2,0);
  r(0,-1) = val/denominatorOp6;
  
  
  val = 24.0 * rr(old_noX-3,old_noY-2)
    +16.0 * (rr(old_noX-4,old_noY-2) + rr(old_noX-2,old_noY-2))
    + 4.0 * (rr(old_noX-5,old_noY-2) + rr(old_noX-4,old_noY-3) + rr(old_noX-2,old_noY-3))
    + 6.0 *  rr(old_noX-3,old_noY-3)
    +       rr(old_noX-5,old_noY-3);
  r(noX-3,noY-2) = val/denominatorOp6;
  
  
  val = 24.0 * rr(old_noX-2,old_noY-3)
    +16.0 * (rr(old_noX-2,old_noY-2) + rr(old_noX-2,old_noY-4))
    + 4.0 * (rr(old_noX-3,old_noY-2) + rr(old_noX-3,old_noY-4) + rr(old_noX-2,old_noY-5))
    + 6.0 *  rr(old_noX-3,old_noY-3)
    +       rr(old_noX-3,old_noY-5);
  r(noX-2,noY-3) = val/denominatorOp6;
  
  
  val = 24.0 * rr(old_noX-3,-1)
    +16.0 * (rr(old_noX-4,-1) + rr(old_noX-2,-1))
    + 4.0 * (rr(old_noX-5,-1) + rr(old_noX-4,0) + rr(old_noX-2,0))
    + 6.0 *  rr(old_noX-3,0)
    +       rr(old_noX-5,0);
  r(noX-3,-1) = val/denominatorOp6;
  
  val = 24.0 * rr(old_noX-2,0)
    +16.0 * (rr(old_noX-2,-1) + rr(old_noX-2,1))
    + 4.0 * (rr(old_noX-3,-1) + rr(old_noX-3,1) + rr(old_noX-2,2))
    + 6.0 *  rr(old_noX-3,0)
    +       rr(old_noX-3,2);
  r(noX-2,0) = val/denominatorOp6;

  return true;
}
 
