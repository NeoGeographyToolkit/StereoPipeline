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

#include "checkWIN32andSGI.h"

#include <UCBsplineSurface.h>
#include <UCBsplines.h>

#ifdef WIN32
#define WIN32ORSGI
#endif

#ifdef SGI
#define WIN32ORSGI
#endif

#ifdef SGI6
#define WIN32ORSGI
#endif

using namespace UCBspl;


SplineSurface::SplineSurface(boost::shared_ptr<GenMatrixType> PHI,
                             double umin, double vmin, 
                             double umax, double vmax) {
  PHI_  = PHI;
  umin_ = umin;
  vmin_ = vmin;
  umax_ = umax;
  vmax_ = vmax;
}


SplineSurface::SplineSurface(const SplineSurface& surf) {
  PHI_  = surf.PHI_;
  umin_ = surf.umin_;
  vmin_ = surf.vmin_;
  umax_ = surf.umax_;
  vmax_ = surf.vmax_;
}

void SplineSurface::init(boost::shared_ptr<GenMatrixType> PHI,
                         double umin, double vmin, 
                         double umax, double vmax) {
  PHI_  = PHI;
  umin_ = umin;
  vmin_ = vmin;
  umax_ = umax;
  vmax_ = vmax;
}


double SplineSurface::f(double u, double v) const { 
  


  int m_ = PHI_->noX()-3;
  int n_ = PHI_->noY()-3;
  double uc = (u - umin_)/(umax_-umin_) * (double)m_;
  double vc = (v - vmin_)/(vmax_-vmin_) * (double)n_;
  
  int i, j;
  double s, t;
  UCBspl::ijst(m_, n_, uc, vc, i, j, s, t);

  double Bks[4];
  double Blt[4];

  Bks[0] = UCBspl::B_0(s); Blt[0] = UCBspl::B_0(t);
  Bks[1] = UCBspl::B_1(s); Blt[1] = UCBspl::B_1(t);
  Bks[2] = UCBspl::B_2(s); Blt[2] = UCBspl::B_2(t);
  Bks[3] = UCBspl::B_3(s); Blt[3] = UCBspl::B_3(t);



  double val = 0.0;
  for (int k = 0; k <= 3; k++)
    for (int l = 0; l <= 3; l++)
      val += (*PHI_)(i+k,j+l)*Bks[k]*Blt[l];
        
  return val;
}


double SplineSurface::f(int ii, int jj) const {
  
#ifdef  UNIFORM_CUBIC_C1_SPLINES
  int i = 2*ii - 1;
  int j = 2*jj - 1;
  int m_size = 2;
#else
  int i = ii - 1;
  int j = jj - 1;
  int m_size = 3;
#endif
      
  double val = 0.0;
  for (int k = 0; k < m_size; k++) {
    for (int l = 0; l < m_size; l++) {
      val += (*PHI_)(i+k,j+l)*UCBspl::tensor_BB[k][l];
    }
  }

  return val;
}


void SplineSurface::normalVector(int ii, int jj, double& gx, double& gy, double& gz) const { 

#ifdef  UNIFORM_CUBIC_C1_SPLINES
  int i = 2*ii - 1;
  int j = 2*jj - 1;
  int m_size = 2;
  int incr = 1;
#else
  int i = ii - 1;
  int j = jj - 1;
  int m_size = 3; 
  int incr = 2;
#endif

  int k,l;
  double val1 = 0.0;
  for (k = 0; k < m_size; k+=incr) {
    for (l = 0; l < m_size; l++) {
      val1 += (*PHI_)(i+k,j+l) * UCBspl::tensor_dBB[k][l];
    }
  }
  double val2 = 0.0;
  for (k = 0; k < m_size; k++) {
    for (l = 0; l < m_size; l+=incr) {
      val2 += (*PHI_)(i+k,j+l) * UCBspl::tensor_BdB[k][l];
    }
  }

  int m_ = PHI_->noX()-3;
  int n_ = PHI_->noY()-3;
  val1 *= ((double)m_/(umax_-umin_));
  val2 *= ((double)n_/(vmax_-vmin_));

  double len = sqrt(val1*val1 + val2*val2 + 1.0);

  gx = -(val1/len);
  gy = -(val2/len);
  gz = 1.0/len;
}

void SplineSurface::derivatives(double u, double v, double& dx, double& dy) const { 
  
  int m_ = PHI_->noX()-3;
  int n_ = PHI_->noY()-3;
  double uc = (u - umin_)/(umax_-umin_) * (double)m_;
  double vc = (v - vmin_)/(vmax_-vmin_) * (double)n_;
  
  int i, j;
  double s, t;
  UCBspl::ijst(m_, n_, uc, vc, i, j, s, t);
  
  double  Bks[4];
  double  Blt[4];
  double dBks[4];
  double dBlt[4];
  
    Bks[0] = UCBspl::B_0(s); Blt[0] = UCBspl::B_0(t);
    Bks[1] = UCBspl::B_1(s); Blt[1] = UCBspl::B_1(t);
    Bks[2] = UCBspl::B_2(s); Blt[2] = UCBspl::B_2(t);
    Bks[3] = UCBspl::B_3(s); Blt[3] = UCBspl::B_3(t);


    dBks[0] = UCBspl::dB_0(s); dBlt[0] = UCBspl::dB_0(t);
    dBks[1] = UCBspl::dB_1(s); dBlt[1] = UCBspl::dB_1(t);
    dBks[2] = UCBspl::dB_2(s); dBlt[2] = UCBspl::dB_2(t);
    dBks[3] = UCBspl::dB_3(s); dBlt[3] = UCBspl::dB_3(t);


  
  double val1 = 0.0;
  double val2 = 0.0;
  
  for (int k = 0; k <= 3; k++) {
    for (int l = 0; l <= 3; l++) {
      val1 += (*PHI_)(i+k,j+l) * dBks[k]*  Blt[l];
      val2 += (*PHI_)(i+k,j+l) *  Bks[k] * dBlt[l];
    }
  }
  
  val1 *= ((double)m_/(umax_-umin_));
  val2 *= ((double)n_/(vmax_-vmin_));
  
  dx = val1;
  dy = val2;
  
}

void SplineSurface::secondDerivatives(double u, double v, double& ddx, double& ddy, double& dxdy) const { 
  
  int m_ = PHI_->noX()-3;
  int n_ = PHI_->noY()-3;
  double uc = (u - umin_)/(umax_-umin_) * (double)m_;
  double vc = (v - vmin_)/(vmax_-vmin_) * (double)n_;
  
  int i, j;
  double s, t;
  UCBspl::ijst(m_, n_, uc, vc, i, j, s, t);
  
  double   Bks[4];
  double   Blt[4];
  double  dBks[4];
  double  dBlt[4];
  double ddBks[4];
  double ddBlt[4];
  
  Bks[0] = UCBspl::B_0(s); Blt[0] = UCBspl::B_0(t);
  Bks[1] = UCBspl::B_1(s); Blt[1] = UCBspl::B_1(t);
  Bks[2] = UCBspl::B_2(s); Blt[2] = UCBspl::B_2(t);
  Bks[3] = UCBspl::B_3(s); Blt[3] = UCBspl::B_3(t);

  dBks[0] = UCBspl::dB_0(s); dBlt[0] = UCBspl::dB_0(t);
  dBks[1] = UCBspl::dB_1(s); dBlt[1] = UCBspl::dB_1(t);
  dBks[2] = UCBspl::dB_2(s); dBlt[2] = UCBspl::dB_2(t);
  dBks[3] = UCBspl::dB_3(s); dBlt[3] = UCBspl::dB_3(t);

  ddBks[0] = UCBspl::ddB_0(s); ddBlt[0] = UCBspl::ddB_0(t);
  ddBks[1] = UCBspl::ddB_1(s); ddBlt[1] = UCBspl::ddB_1(t);
  ddBks[2] = UCBspl::ddB_2(s); ddBlt[2] = UCBspl::ddB_2(t);
  ddBks[3] = UCBspl::ddB_3(s); ddBlt[3] = UCBspl::ddB_3(t);



  
  double val1 = 0.0;
  double val2 = 0.0;
  double val3 = 0.0;
  
  for (int k = 0; k <= 3; k++) {
    for (int l = 0; l <= 3; l++) {
      val1 += (*PHI_)(i+k,j+l) * ddBks[k] *   Blt[l];
      val2 += (*PHI_)(i+k,j+l) *   Bks[k] * ddBlt[l];
      val3 += (*PHI_)(i+k,j+l) *  dBks[k] *  dBlt[l];
    }
  }
  
  
  val1 *= ((double)m_/(umax_-umin_));
  val2 *= ((double)n_/(vmax_-vmin_));
  
  ddx = val1;
  ddy = val2;
  dxdy = val3;
  
}

void SplineSurface::curvatures(double u, double v, double& profC, double& planC) const { 
  
  double ddx;
  double ddy;
  double dxdy;
  
  secondDerivatives(u,v,ddx,ddy,dxdy);
  
  double gx;
  double gy;
  double gz;
  
  normalVector(u,v,gx,gy,gz);
  
  double len = sqrt(1-gz*gz);
  
  double u1 = gx/len;
  double u2 = gy/len;
  
  double v1 = -gy/len;
  double v2 = gx/len; 
  
  
  double dirDDProfC = u1*u1*ddx + 2*u1*u2*dxdy + u2*u2*ddy;
  double dirDDPlanC = v1*v1*ddx + 2*v1*v2*dxdy + v2*v2*ddy;
  
  
  profC = dirDDProfC/((1+(dirDDProfC)*(dirDDProfC))*sqrt(1+(dirDDProfC)*(dirDDProfC)));
  planC = dirDDPlanC/((1+(dirDDPlanC)*(dirDDPlanC))*sqrt(1+(dirDDPlanC)*(dirDDPlanC)));
  
}

void SplineSurface::eval(int i, int j, double& z, double& gx, double& gy, double& gz) const {

  z = f(i,j);
  normalVector(i, j, gx, gy, gz);
}

void SplineSurface::normalVector(double u, double v, double& gx, double& gy, double& gz) const { 
  
  int m_ = PHI_->noX()-3;
  int n_ = PHI_->noY()-3;
  double uc = (u - umin_)/(umax_-umin_) * (double)m_;
  double vc = (v - vmin_)/(vmax_-vmin_) * (double)n_;
  
  int i, j;
  double s, t;
  UCBspl::ijst(m_, n_, uc, vc, i, j, s, t);
  
  double  Bks[4];
  double  Blt[4];
  double dBks[4];
  double dBlt[4];

  Bks[0] = UCBspl::B_0(s); Blt[0] = UCBspl::B_0(t);
  Bks[1] = UCBspl::B_1(s); Blt[1] = UCBspl::B_1(t);
  Bks[2] = UCBspl::B_2(s); Blt[2] = UCBspl::B_2(t);
  Bks[3] = UCBspl::B_3(s); Blt[3] = UCBspl::B_3(t);

  dBks[0] = UCBspl::dB_0(s); dBlt[0] = UCBspl::dB_0(t);
  dBks[1] = UCBspl::dB_1(s); dBlt[1] = UCBspl::dB_1(t);
  dBks[2] = UCBspl::dB_2(s); dBlt[2] = UCBspl::dB_2(t);
  dBks[3] = UCBspl::dB_3(s); dBlt[3] = UCBspl::dB_3(t);



  
  double val1 = 0.0;
  double val2 = 0.0;
  for (int k = 0; k <= 3; k++) {
    for (int l = 0; l <= 3; l++) {
      val1 += (*PHI_)(i+k,j+l) * dBks[k]*  Blt[l];
      val2 += (*PHI_)(i+k,j+l) *  Bks[k] * dBlt[l];
    }
  }
  
  val1 *= ((double)m_/(umax_-umin_));
  val2 *= ((double)n_/(vmax_-vmin_));

  double len = sqrt(val1*val1 + val2*val2 + 1.0);

  gx = -(val1/len);
  gy = -(val2/len);
  gz = 1.0/len;
}

void SplineSurface::eval(double u, double v, double& z, double& gx, double& gy, double& gz) const {

  int m_ = PHI_->noX()-3;
  int n_ = PHI_->noY()-3;
  double uc = (u - umin_)/(umax_-umin_) * (double)m_;
  double vc = (v - vmin_)/(vmax_-vmin_) * (double)n_;
  
  int i, j;
  double s, t;
  UCBspl::ijst(m_, n_, uc, vc, i, j, s, t);
  
  double Bks[4];
  double Blt[4];
  double dBks[4];
  double dBlt[4];

  Bks[0] = UCBspl::B_0(s); Blt[0] = UCBspl::B_0(t);
  Bks[1] = UCBspl::B_1(s); Blt[1] = UCBspl::B_1(t);
  Bks[2] = UCBspl::B_2(s); Blt[2] = UCBspl::B_2(t);
  Bks[3] = UCBspl::B_3(s); Blt[3] = UCBspl::B_3(t);

  dBks[0] = UCBspl::dB_0(s); dBlt[0] = UCBspl::dB_0(t);
  dBks[1] = UCBspl::dB_1(s); dBlt[1] = UCBspl::dB_1(t);
  dBks[2] = UCBspl::dB_2(s); dBlt[2] = UCBspl::dB_2(t);
  dBks[3] = UCBspl::dB_3(s); dBlt[3] = UCBspl::dB_3(t);



  double val1 = 0.0;
  double val2 = 0.0;

  z = 0.0;
  for (int k = 0; k <= 3; k++) {
    for (int l = 0; l <= 3; l++) {
      z    += (*PHI_)(i+k,j+l) *  Bks[k] *  Blt[l];
      val1 += (*PHI_)(i+k,j+l) * dBks[k] *  Blt[l];
      val2 += (*PHI_)(i+k,j+l) *  Bks[k] * dBlt[l];
    }
  }

  val1 *= ((double)m_/(umax_-umin_));
  val2 *= ((double)n_/(vmax_-vmin_));
  
  double len = sqrt(val1*val1 + val2*val2 + 1.0);

  gx = -(val1/len);
  gy = -(val2/len);
  gz = 1.0/len;
}

void SplineSurface::refineCoeffs() {

  GenMatrix<UCBspl_real>* PHIrefined = new GenMatrix<UCBspl_real>();
  UCBspl::refineCoeffsC2(*PHI_, *PHIrefined);	
  PHI_.reset(PHIrefined);
}

bool SplineSurface::restrictCoeffs() {

  GenMatrix<UCBspl_real>* PHIrestricted = new GenMatrix<UCBspl_real>();
  bool status = UCBspl::restrictCoeffsC2(*PHI_, *PHIrestricted);	
  if (!status)
    return status;
  PHI_.reset(PHIrestricted);

  return true;
}    
 
